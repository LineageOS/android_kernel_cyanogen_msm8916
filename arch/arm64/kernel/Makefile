#
# Makefile for the linux kernel.
#

CPPFLAGS_vmlinux.lds	:= -DTEXT_OFFSET=$(TEXT_OFFSET)
AFLAGS_head.o		:= -DTEXT_OFFSET=$(TEXT_OFFSET)
CFLAGS_efi-stub.o 	:= -DTEXT_OFFSET=$(TEXT_OFFSET) \
			   -I$(src)/../../../scripts/dtc/libfdt

CFLAGS_REMOVE_ftrace.o = -pg
CFLAGS_REMOVE_insn.o = -pg
CFLAGS_REMOVE_return_address.o = -pg

# Object file lists.
arm64-obj-y		:= cputable.o debug-monitors.o irq.o fpsimd.o	\
			   process.o ptrace.o setup.o signal.o	\
			   sys.o stacktrace.o time.o traps.o io.o vdso.o	\
			   cpu_pwr_ctl.o msm_cpu_ops.o psci.o cpu_ops.o insn.o opcodes.o		\
			   return_address.o #hyp-stub.o entry-fpsimd.o entry.o 

arm64-obj-$(CONFIG_COMPAT)		+= signal32.o		\
					   sys_compat.o #sys32.o kuser32.o 
arm64-obj-$(CONFIG_FUNCTION_TRACER)	+= ftrace.o entry-ftrace.o
arm64-obj-$(CONFIG_MODULES)		+= arm64ksyms.o module.o
arm64-obj-$(CONFIG_SMP)			+= smp.o smp_spin_table.o topology.o
arm64-obj-$(CONFIG_PERF_EVENTS)		+= perf_regs.o
arm64-obj-$(CONFIG_HW_PERF_EVENTS)	+= perf_event.o perf_debug.o 		\
					   perf_trace_counters.o
arm64-obj-$(CONFIG_HAVE_HW_BREAKPOINT)	+= hw_breakpoint.o
arm64-obj-$(CONFIG_EARLY_PRINTK)	+= early_printk.o
arm64-obj-$(CONFIG_ARM64_CPU_SUSPEND)	+= suspend.o #sleep.o 
arm64-obj-$(CONFIG_JUMP_LABEL)		+= jump_label.o
arm64-obj-$(CONFIG_KGDB)		+= kgdb.o
arm64-obj-$(CONFIG_EFI)			+= efi.o efi-stub.o efi-entry.o

obj-$(CONFIG_SWP_EMULATE)		+= swp_emulate.o

obj-$(CONFIG_PCI)			+= pcibios.o
obj-y					+= $(arm64-obj-y) vdso/
obj-m					+= $(arm64-obj-m)
#sorry for this shit
head-y		:= 

head-y		+= ../crypto/aes-ce-ccm-core.o
head-y		+= ../crypto/ghash-ce-core.o
head-y		+= ../crypto/sha2-ce-core.o
head-y		+= ../crypto/aes-ce.o
head-y		+= ../crypto/aes-neon.o
head-y		+= ../crypto/sha1-ce-core.o

head-y		+= entry-fpsimd.o
head-y		+= entry.o
head-y		+= hyp-stub.o
head-y		+= sleep.o
head-y		+= kuser32.o
head-y		+= sys32.o
head-y		+= head.o
head-y		+= idle-v8.o

head-y		+= vdso-wrap.o

head-y		+= ../mm/cache.o
head-y		+= ../mm/proc.o
extra-y					:= $(head-y) vmlinux.lds

# vDSO - this must be built first to generate the symbol offsets
$(call objectify,$(arm64-obj-y)): $(obj)/vdso/vdso-offsets.h
$(obj)/vdso/vdso-offsets.h: $(obj)/vdso
