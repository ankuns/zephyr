# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(sys_clock_init.c)
zephyr_library_sources_ifdef(CONFIG_ALTERA_AVALON_TIMER altera_avalon_timer_hal.c)
zephyr_library_sources_ifdef(CONFIG_AMBIQ_STIMER_TIMER ambiq_stimer.c)
zephyr_library_sources_ifdef(CONFIG_APIC_TIMER apic_timer.c)
zephyr_library_sources_ifdef(CONFIG_APIC_TSC_DEADLINE_TIMER apic_tsc.c)
zephyr_library_sources_ifdef(CONFIG_ARCV2_TIMER arcv2_timer0.c)
zephyr_library_sources_ifdef(CONFIG_ARM_ARCH_TIMER arm_arch_timer.c)
zephyr_library_sources_ifdef(CONFIG_INTEL_ADSP_TIMER intel_adsp_timer.c)
zephyr_library_sources_ifdef(CONFIG_CC13XX_CC26XX_RTC_TIMER cc13xx_cc26xx_rtc_timer.c)
zephyr_library_sources_ifdef(CONFIG_CORTEX_M_SYSTICK cortex_m_systick.c)
zephyr_library_sources_ifdef(CONFIG_ESP32C3_SYS_TIMER esp32c3_sys_timer.c)
zephyr_library_sources_ifdef(CONFIG_GECKO_BURTC_TIMER gecko_burtc_timer.c)
zephyr_library_sources_ifdef(CONFIG_HPET_TIMER hpet.c)
zephyr_library_sources_ifdef(CONFIG_ITE_IT8XXX2_TIMER ite_it8xxx2_timer.c)
zephyr_library_sources_ifdef(CONFIG_LEON_GPTIMER leon_gptimer.c)
zephyr_library_sources_ifdef(CONFIG_LITEX_TIMER litex_timer.c)
zephyr_library_sources_ifdef(CONFIG_MCHP_XEC_RTOS_TIMER mchp_xec_rtos_timer.c)
zephyr_library_sources_ifdef(CONFIG_MCUX_LPTMR_TIMER mcux_lptmr_timer.c)
zephyr_library_sources_ifdef(CONFIG_MCUX_OS_TIMER mcux_os_timer.c)
zephyr_library_sources_ifdef(CONFIG_MCUX_GPT_TIMER mcux_gpt_timer.c)
zephyr_library_sources_ifdef(CONFIG_MIPS_CP0_TIMER mips_cp0_timer.c)
zephyr_library_sources_ifdef(CONFIG_NATIVE_POSIX_TIMER native_posix_timer.c)
zephyr_library_sources_ifdef(CONFIG_NPCX_ITIM_TIMER npcx_itim_timer.c)


# Function for providing compile-time definition which value is calculated
# by processing Kconfig options.
#
# This function is intended to calculate capacities of some resources needed
# by modules scattered among Zephyr and other projects even unknown to Zephyr.
# Provides possibility inverse dependency between modules while still allowing
# to have value calculated in compile-time.
#
# Usage:
#   zephyr_compile_definition_kconfig_based(<out-definition> <base-kconfig>
#           [SUFFIX_FOR_ADD <suffix>]
#           [SUFFIX_FOR_MIN <suffix>]
#   )
#
# <out-definition>:          Output macro definition that will be provided to the build.
#                            This macro is calculated by this function based on
#                            initial value of <base-kconfig> and any other Kconfig options
#                            that can increase the value of out-definition.
#
# <base-kconfig>:            The Kconfig option that will be used as an initial value
#                            and prefix for all other Kconfigs that can be used by
#                            other modules to impose requirements on <out-definition>.
#
# SUFFIX_FOR_ADD <suffix>:   Optional. Part of Kconfig identifiers used for incrementing
#                            the value of out-definition. By convention use suffix=ADD
#                            If necessary for legacy purposes suffix can have other value.
#                            Kconfigs for increasing total value of <out-definition>
#                            match pattern `<base-kconfig>_<suffix>_*`.
#
# SUFFIX_FOR_MIN <suffix>:   Optional. Part of Kconfig identifiers imposing a limit
#                            on minimum value of out-definition. By convention use suffix=MIN
#                            If necessary for legacy purposes the suffix can have other value.
#                            Kconfigs imposing the limit match pattern
#                            `<base-kconfig>_<suffix>_*`
#
# Examples:
#
# Consider hypotethical <some resource> which provides a Kconfig SOME_RESOURCE_CAPACITY
#
# config SOME_RESOURCE_CAPACITY
#     int "<some resource> capatity"
#     default 0
#     help
#       Capacity of <some resource>.
#       The total capacity is given by K_SOME_RESOURCE_CAPACITY macro which is calculated
#       automatically during the build based on:
#       - the SOME_RESOURCE_CAPACITY option itself which is the initial value.
#       - the SOME_RESOURCE_CAPACITY_ADD_* options that can be scattered among modules.
#       - the SOME_RESOURCE_CAPACITY_MIN_* options that can be scattered among modules.
#
#       The Kconfig options named according to pattern SOME_RESOURCE_CAPACITY_ADD_*
#       can be used by any module. They are individual requirements on the capacity
#       of <some resource>. The K_SOME_RESOURCE_CAPACITY macro is increased by each
#       of the found Kconfig option SOME_RESOURCE_CAPACITY_ADD_*.
#
#       The Kconfig options named SOME_RESOURCE_CAPACITY_MIN_* set the requirement
#       on minimum value of K_SOME_RESOURCE_CAPACITY macro regardless of other requirements.
#       K_SOME_RESOURCE_CAPACITY is guaranteed to be no less than the maximum value of
#       specified minimum values.
#
# Then consider following Kconfig options scattered among Zephyr or other repositories
# which can be not known to the Zephyr but known when building.
#
# config SOME_RESOURCE_CAPACITY_ADD_FOO
#     int
#     default 10
#
# config SOME_RESOURCE_CAPACITY_ADD_BAR
#     int
#     default 5
#
# config SOME_RESOURCE_CAPACITY_ADD_BAZ
#     int
#     default 2
#
# By using following call to the cmake function:
#
#   zephyr_compile_definition_kconfig_based(K_SOME_RESOURCE_CAPACITY
#       CONFIG_SOME_RESOURCE_CAPACITY
#           SUFFIX_FOR_ADD ADD
#           SUFFIX_FOR_ADD MIN
#   )
#
# The macro K_SOME_RESOURCE_CAPACITY will be provided to the build with value 17
# being the sum of CONFIG_SOME_RESOURCE_CAPACITY and found Kconfigs
# CONFIG_SOME_RESOURCE_CAPACITY_ADD_FOO, CONFIG_SOME_RESOURCE_CAPACITY_ADD_BAR,
# CONFIG_SOME_RESOURCE_CAPACITY_ADD_BAZ.
#
# When you add somewhere additional hypothetical Kconfigs:
#
# config SOME_RESOURCE_CAPACITY_MIN_FOO
#     int
#     default 13
#
# the macro K_SOME_RESOURCE_CAPACITY would still get value 17, because the value 13
# is less than calculated value. However if CONFIG_SOME_RESOURCE_CAPACITY_ADD_BAR
# would not be defined, the K_SOME_RESOURCE_CAPACITY would get value 13 imposed
# by the SOME_RESOURCE_CAPACITY_MIN_FOO rather than 12 being the sum of
# CONFIG_SOME_RESOURCE_CAPACITY, CONFIG_SOME_RESOURCE_CAPACITY_ADD_FOO and
# CONFIG_SOME_RESOURCE_CAPACITY_ADD_BAZ.
#
# When you add additional hypothetical Kconfig:
#
# config SOME_RESOURCE_CAPACITY_MIN_BAR
#     int
#     default 20
#
# Then the K_SOME_RESOURCE_CAPACITY will get value 20.
#
function(zephyr_compile_definition_kconfig_based out_definition base_kconfig)

    set(oneValueArgs SUFFIX_FOR_ADD SUFFIX_FOR_MIN)
    cmake_parse_arguments(local_params "" "${oneValueArgs}" "" ${ARGN})

    # Let initial value be given by base_kconfig
    set(total ${${base_kconfig}})

    if(DEFINED local_params_SUFFIX_FOR_ADD)
        set(kconfig_for_add ${base_kconfig}_${local_params_SUFFIX_FOR_ADD}_)
        set(keys_list "")
        import_kconfig(${kconfig_for_add} ${DOTCONFIG} keys_list)

        foreach(vkey ${keys_list})
            math(EXPR total "${total} + ${${vkey}}")
        endforeach()
        set(keys_list "")
    endif()

    if(DEFINED local_params_SUFFIX_FOR_MIN)
        set(kconfig_for_min ${base_kconfig}_${local_params_SUFFIX_FOR_MIN}_)
        set(keys_list "")
        import_kconfig(${kconfig_for_min} ${DOTCONFIG} keys_list)

        foreach(vkey ${keys_list})
            if ( ${${vkey}} GREATER ${total})
                set(total ${${vkey}})
            endif()
        endforeach()
    endif()

    message(STATUS "Calculated definition ${out_definition}=${total}")

    zephyr_compile_definitions(${out_definition}=${total})
endfunction()

if (CONFIG_NRF_RTC_TIMER)
    zephyr_library_sources_ifdef(CONFIG_NRF_RTC_TIMER nrf_rtc_timer.c)

    zephyr_compile_definition_kconfig_based(K_NRF_RTC_TIMER_USER_CHAN_COUNT
        CONFIG_NRF_RTC_TIMER_USER_CHAN_COUNT
           SUFFIX_FOR_ADD "ADD"
           SUFFIX_FOR_MIN "MIN"
    )
endif()

zephyr_library_sources_ifdef(CONFIG_RCAR_CMT_TIMER rcar_cmt_timer.c)
zephyr_library_sources_ifdef(CONFIG_RISCV_MACHINE_TIMER riscv_machine_timer.c)
zephyr_library_sources_ifdef(CONFIG_RV32M1_LPTMR_TIMER rv32m1_lptmr_timer.c)
zephyr_library_sources_ifdef(CONFIG_SAM0_RTC_TIMER sam0_rtc_timer.c)
zephyr_library_sources_ifdef(CONFIG_STM32_LPTIM_TIMER stm32_lptim_timer.c)
zephyr_library_sources_ifdef(CONFIG_XLNX_PSTTC_TIMER xlnx_psttc_timer.c)
zephyr_library_sources_ifdef(CONFIG_XTENSA_TIMER xtensa_sys_timer.c)
