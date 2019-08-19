################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Example_2802xSpi_FFDLB.obj: ../Example_2802xSpi_FFDLB.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.7/bin/cl2000" -v28 -ml -mt --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.7/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2802x/v230/f2802x_common/include" --include_path="C:/ti/controlSUITE/device_support/f2802x/v230/f2802x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2802x/v230" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v15c/include" -g --define="_DEBUG" --define="_FLASH" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="Example_2802xSpi_FFDLB.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


