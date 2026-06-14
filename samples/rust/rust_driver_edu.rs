// SPDX-License-Identifier: GPL-2.0

//! Rust EDU driver sample (based on QEMU's `edu`).
//!
//! To make this driver probe, QEMU must be run with `-device edu`.

use kernel::{
    bitfield,
    device::Bound,
    dma::{
        Coherent,
        Device,
        DmaMask, //
    },
    io::{
        io_project,
        io_write,
        poll::read_poll_timeout,
        register,
        register::Array,
        Io, //
    },
    irq::{
        self,
        Flags, //
    },
    num::casts::IntoSafeCast,
    pci,
    prelude::*,
    sync::Completion,
    time::Delta, //
};

const QEMU_EDU_DEVICE_ID: u32 = 0x11e8;

const IRQ_MAGIC_VALUE: u8 = 42;

mod regs {
    use kernel::io::Region;

    use super::*;

    register! {
        base: Region<{ END }>;

        pub(super) IDENTIFICATION(u32) @ 0x0 {
            31:24 major;
            23:16 minor;
            7:0 magic;
        }

        pub(super) LIVENESS_CHECK(u32) @ 0x04 {}

        pub(super) FACTORIAL(u32) @ 0x08 {}

        pub(super) STATUS(u32) @ 0x20 {
            0:0 computing;
            7:7 raise_interrupt;
        }

        pub(super) IRQ_STATUS(u32) @ 0x24 {
            8:8 dma;
            7:0 test;
        }

        pub(super) IRQ_RAISE(u32) @ 0x60 {
            7:0 test;
        }

        pub(super) IRQ_ACK(u32) @ 0x64 {
            8:8 dma;
            7:0 test;
        }

        pub(super) DMA: Dma @ 0x80;
    }

    impl core::ops::Not for LIVENESS_CHECK {
        type Output = Self;

        fn not(self) -> Self::Output {
            Self::from_raw(!self.into_raw())
        }
    }

    bitfield! {
        #[derive(FromBytes, IntoBytes)]
        pub(super) struct DmaCommand(u64) {
            0:0 start_transfer;
            1:1 direction;
            2:2 raise_irq;
        }
    }

    /// DMA register block. Its 64-bit registers need more alignment than `Region` guarantees, so
    /// project it with `io_project!(bar, try: DMA)`, which checks the alignment at runtime.
    #[repr(C)]
    #[derive(FromBytes, IntoBytes)]
    pub(super) struct Dma {
        pub(super) src: u64,
        pub(super) dst: u64,
        pub(super) count: u64,
        pub(super) command: DmaCommand,
    }

    pub(super) const END: usize = 0xA0;
}

type Bar0<'a> = pci::Bar<'a, { regs::END }>;

struct EduDriver;

#[pin_data(PinnedDrop)]
struct EduDriverData<'bound> {
    pdev: &'bound pci::Device<Bound>,
    #[pin]
    irq_handler: irq::Registration<'vectors, IrqHandler<'vectors>>,
    vectors: pci::IrqVectorRegistration<'bound>,
    // Declared last so the device stays enabled until the IRQ handler is freed.
    _enable: pci::DeviceEnableGuard<'bound>,
}

#[pin_data]
struct IrqHandler<'a> {
    pdev: &'a pci::Device,
    bar: Bar0<'a>,
    #[pin]
    irq_test_completion: Completion,
    #[pin]
    irq_dma_completion: Completion,
    dma: Coherent<'a, u64>,
}

impl EduDriverData<'_> {
    fn selftest(&self) -> Result {
        self.config_space();
        self.magic()?;
        self.liveness_check()?;
        self.factorial()?;
        self.test_irq()?;
        self.test_dma()?;
        Ok(())
    }

    fn config_space(&self) {
        let pdev = self.pdev;
        let config = pdev.config_space();

        // Some PCI configuration space registers.
        register! {
            base: pci::Normal;

            VENDOR_ID(u16) @ 0x0 {
                15:0 vendor_id;
            }

            REVISION_ID(u8) @ 0x8 {
                7:0 revision_id;
            }

            BAR(u32)[6] @ 0x10 {
                31:0 value;
            }
        }

        dev_info!(
            pdev,
            "config space read8 rev ID: {:x}\n",
            config.read(REVISION_ID).revision_id()
        );

        dev_info!(
            pdev,
            "config space read16 vendor ID: {:x}\n",
            config.read(VENDOR_ID).vendor_id()
        );

        dev_info!(
            pdev,
            "config space read32 BAR 0: {:x}\n",
            config.read(BAR::at(0)).value()
        );
    }

    fn magic(&self) -> Result {
        const EXPECTED: regs::IDENTIFICATION =
            regs::IDENTIFICATION::from_raw(0).with_const_magic::<0xed>();

        let pdev = self.pdev;
        let bar = &self.irq_handler().handler().bar;

        let identification = bar.read(regs::IDENTIFICATION);

        if identification.magic() != EXPECTED.magic() {
            dev_err!(
                pdev,
                "magic mismatch: expected {:#x} got {:#x}\n",
                EXPECTED.magic(),
                identification.magic()
            );
            return Err(ENODEV);
        }

        dev_info!(
            pdev,
            "major: {:#x} minor: {:#x}\n",
            identification.major(),
            identification.minor()
        );
        Ok(())
    }

    fn liveness_check(&self) -> Result {
        let pdev = self.pdev;
        let bar = &self.irq_handler().handler().bar;

        let pattern = regs::LIVENESS_CHECK::from_raw(0xdead_beef);
        bar.write_reg(pattern);

        let inversed = bar.read(regs::LIVENESS_CHECK);
        if inversed != !pattern {
            dev_err!(
                pdev,
                "inverse mismatch: expected {:#x} got {:#x}\n",
                !pattern.into_raw(),
                inversed.into_raw()
            );
            return Err(ENODEV);
        }

        dev_info!(pdev, "inverse test successful\n");
        Ok(())
    }

    fn factorial(&self) -> Result {
        let pdev = self.pdev;
        let bar = &self.irq_handler().handler().bar;

        self.wait_until_compute_has_finished()?;

        bar.write(regs::FACTORIAL, 4.into());

        self.wait_until_compute_has_finished()?;

        let result: u32 = bar.read(regs::FACTORIAL).into();

        let expected = 24;

        if result != expected {
            dev_err!(
                pdev,
                "factorial result wrong: expected {} got {}\n",
                expected,
                result
            );
            return Err(ENODEV);
        }

        dev_info!(pdev, "factorial test successful\n");
        Ok(())
    }

    fn test_irq(&self) -> Result {
        let pdev = self.pdev;
        let handler = self.irq_handler().handler();

        dev_dbg!(pdev, "raising irq\n");

        handler.bar.write(
            regs::IRQ_RAISE,
            regs::IRQ_RAISE::zeroed().with_test(IRQ_MAGIC_VALUE),
        );

        handler.irq_test_completion.wait_for_completion();

        dev_info!(pdev, "irq test successful\n");
        Ok(())
    }

    fn test_dma(&self) -> Result {
        let pdev = self.pdev;
        let handler = self.irq_handler().handler();

        const QEMU_DMA_BASE: u64 = 0x40000;

        dev_dbg!(pdev, "testing dma\n");

        let dma = &handler.dma;

        const DMA_VALUE: u64 = 42;

        dma.write_val(DMA_VALUE);

        let dma_regs = io_project!(handler.bar, try: regs::DMA);

        io_write!(dma_regs, .src, dma.dma_address());
        io_write!(dma_regs, .dst, QEMU_DMA_BASE);
        io_write!(dma_regs, .count, dma.size().into_safe_cast());
        io_write!(
            dma_regs,
            .command,
            regs::DmaCommand::zeroed()
                .with_start_transfer(true)
                .with_direction(false)
                .with_raise_irq(true)
        );

        handler.irq_dma_completion.wait_for_completion();

        // Destroy previous value to test roundtrip
        dma.write_val(0);

        io_write!(dma_regs, .src, QEMU_DMA_BASE);
        io_write!(dma_regs, .dst, dma.dma_address());
        io_write!(dma_regs, .count, dma.size().into_safe_cast());
        io_write!(
            dma_regs,
            .command,
            regs::DmaCommand::zeroed()
                .with_start_transfer(true)
                .with_direction(true)
                .with_raise_irq(true)
        );

        handler.irq_dma_completion.wait_for_completion();

        let result = dma.read_val();

        if result != DMA_VALUE {
            dev_err!(
                pdev,
                "dma result wrong: expected {} got {}\n",
                DMA_VALUE,
                result
            );
            return Err(ENODEV);
        }

        dev_info!(pdev, "dma test successful\n");
        Ok(())
    }

    fn wait_until_compute_has_finished(&self) -> Result {
        let pdev = self.pdev;
        let bar = &self.irq_handler().handler().bar;

        read_poll_timeout(
            || Ok(bar.read(regs::STATUS)),
            |status| status.computing() == 0,
            Delta::from_millis(10),
            Delta::from_millis(100),
        )
        .inspect_err(|_| dev_err!(pdev, "computation bit did not clear before timeout\n"))
        .map(|_| ())
    }
}

impl pci::Driver for EduDriver {
    type IdInfo = ();
    type Data<'bound> = EduDriverData<'bound>;

    const ID_TABLE: pci::IdTable<Self::IdInfo> = &PCI_TABLE;

    fn probe<'bound>(
        pdev: &'bound pci::Device<kernel::device::Core<'_>>,
        _id_info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        try_pin_init!(EduDriverData {
            _: {
                dev_dbg!(
                    pdev,
                    "Probe Rust EDU driver sample (PCI ID: {}, 0x{:x}).\n",
                    pdev.vendor_id(),
                    pdev.device_id()
                );
            },

            _enable: {
                let enable = pdev.enable_device()?;
                pdev.set_master(&enable)?;
                enable
            },

            _: {
                // SAFETY: There are no concurrent calls to DMA allocation and mapping primitives.
                unsafe { pdev.dma_set_mask_and_coherent(DmaMask::new::<28>())? };
            },

            vectors: pdev
                .alloc_irq_vectors(1, 1, pci::IrqType::Msi.into())
                .inspect_err(|e| dev_err!(pdev, "alloc_irq_vectors failed: {:?}\n", e))?,

            // SAFETY: `irq_handler` is owned by `EduDriverData`, which the driver core drops on
            // unbind, so the Registration is not leaked.
            irq_handler <- unsafe {
                irq::Registration::new(
                    vectors.index(0)?.into(),
                    Flags::TRIGGER_NONE,
                    c"rust_edu_irq",
                    try_pin_init!(IrqHandler {
                        bar: pdev.iomap_region_sized(0, c"rust_driver_edu")?,
                        // Ack interrupts left pending by a previous binding.
                        _: {
                            let status = bar.read(regs::IRQ_STATUS);
                            bar.write(regs::IRQ_ACK, regs::IRQ_ACK::from_raw(status.into_raw()));
                        },
                        irq_test_completion <- Completion::new(),
                        irq_dma_completion <- Completion::new(),
                        dma: Coherent::zeroed(pdev.as_ref(), GFP_KERNEL)?,
                        pdev,
                    }? Error),
                )
            },

            pdev,
        })
        .pin_chain(|this| {
            this.selftest()?;
            dev_info!(this.pdev, "rust_driver_edu successfully initialized\n");
            Ok(())
        })
    }
}

impl irq::Handler for IrqHandler<'_> {
    fn handle(&self) -> irq::IrqReturn {
        dev_dbg!(self.pdev, "irq handler called\n");
        let status = self.bar.read(regs::IRQ_STATUS);
        let mut ret = irq::IrqReturn::None;

        if status.dma() != 0 {
            dev_dbg!(self.pdev, "handling dma completion in irq\n");
            self.bar
                .write(regs::IRQ_ACK, regs::IRQ_ACK::zeroed().with_dma(true));
            self.irq_dma_completion.complete();
            ret = irq::IrqReturn::Handled;
        }

        if u8::from(status.test()) == IRQ_MAGIC_VALUE {
            dev_dbg!(self.pdev, "handling test completion in irq\n");
            self.bar.write(
                regs::IRQ_ACK,
                regs::IRQ_ACK::zeroed().with_test(IRQ_MAGIC_VALUE),
            );
            self.irq_test_completion.complete();
            ret = irq::IrqReturn::Handled;
        }

        ret
    }
}

#[pinned_drop]
impl PinnedDrop for EduDriverData<'_> {
    fn drop(self: Pin<&mut Self>) {
        dev_dbg!(self.pdev, "Remove Rust EDU driver sample.\n");
    }
}

kernel::pci_device_table!(
    PCI_TABLE,
    <EduDriver as pci::Driver>::IdInfo,
    [(
        pci::DeviceId::from_id(pci::Vendor::QEMU, QEMU_EDU_DEVICE_ID),
        ()
    )]
);

kernel::module_pci_driver! {
    type: EduDriver,
    name: "rust_driver_edu",
    authors: ["Maurice Hieronymus"],
    description: "Rust EDU driver",
    license: "GPL v2",
}
