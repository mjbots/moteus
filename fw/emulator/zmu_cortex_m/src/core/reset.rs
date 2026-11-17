//!
//! Processor Reset logic
//!

use crate::Processor;
use crate::ProcessorMode;
use crate::bus::Bus;
use crate::core::exception::ExceptionHandling;
use crate::core::fault::Fault;
use crate::core::register::{BaseReg, PSR};

/// Trait for processor reset
pub trait Reset {
    ///
    /// Reset Processor
    ///
    fn reset(&mut self) -> Result<(), Fault>;
}

impl Reset for Processor {
    fn reset(&mut self) -> Result<(), Fault> {
        // All basic registers to zero.
        for r in &mut self.r0_12 {
            *r = 0;
        }

        // Main stack pointer is read via vector table
        let vtor = self.vtor;
        let sp = self.read32(vtor).map_err(Fault::on_vector_read)? & 0xffff_fffc;
        self.set_msp(sp);

        // Process stack pointer to zero
        self.set_psp(0);

        // Link Register
        self.lr = 0;

        // Mode
        self.mode = ProcessorMode::ThreadMode;

        // Apsr, ipsr
        self.psr = PSR { value: 0 };
        self.primask = false;

        #[cfg(not(feature = "armv6m"))]
        {
            self.faultmask = false;
            self.basepri = 0;
        }
        self.control.sp_sel = false;
        self.control.n_priv = false;
        #[cfg(feature = "has-fp")]
        {
            self.control.fpca = false;
        }

        //TODO self.scs.reset();
        self.exceptions_reset();
        self.reset_scb_fault_state();
        self.reset_fp_system_state();

        //self.event_reg.clear();

        self.itstate = 0;
        self.execution_priority = self.get_execution_priority();

        let reset_vector = self.read32(vtor + 4).map_err(Fault::on_vector_read)?;
        self.blx_write_pc(reset_vector);
        Ok(())
    }
}
