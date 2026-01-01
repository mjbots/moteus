#!/usr/bin/env python3

# Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Generate GDB convenience variables for STM32 peripherals from an SVD file.

Usage:
    python3 generate_gdb_peripherals.py <svd_file> [output_file]

If output_file is not specified, writes to stdout.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# Map SVD groupName to C typedef name
# These match the CMSIS/mbed type definitions
# Only include types that are likely to be in debug symbols
GROUP_TO_TYPE = {
    'ADC': 'ADC_TypeDef',
    'CORDIC': 'CORDIC_TypeDef',
    'DAC': 'DAC_TypeDef',
    'DMA': 'DMA_TypeDef',
    'DMAMUX': 'DMAMUX_Channel_TypeDef',
    'EXTI': 'EXTI_TypeDef',
    'Flash': 'FLASH_TypeDef',
    'GPIO': 'GPIO_TypeDef',
    'I2C': 'I2C_TypeDef',
    'LPTIM': 'LPTIM_TypeDef',
    'RCC': 'RCC_TypeDef',
    'SPI': 'SPI_TypeDef',
    'SYSCFG': 'SYSCFG_TypeDef',
    'TIM': 'TIM_TypeDef',
    'USART': 'USART_TypeDef',
}

# Types that may or may not be present depending on firmware usage
# These are tried but won't cause errors if missing
OPTIONAL_TYPES = {
    'COMP': 'COMP_TypeDef',
    'CRC': 'CRC_TypeDef',
    'CRS': 'CRS_TypeDef',
    'DBGMCU': 'DBGMCU_TypeDef',
    'FDCAN': 'FDCAN_GlobalTypeDef',
    'FMAC': 'FMAC_TypeDef',
    'FMC': 'FMC_Bank1_TypeDef',
    'HRTIM': 'HRTIM_TypeDef',
    'IWDG': 'IWDG_TypeDef',
    'OPAMP': 'OPAMP_TypeDef',
    'PWR': 'PWR_TypeDef',
    'QUADSPI': 'QUADSPI_TypeDef',
    'RNG': 'RNG_TypeDef',
    'RTC': 'RTC_TypeDef',
    'SAI': 'SAI_TypeDef',
    'TAMP': 'TAMP_TypeDef',
    'UCPD': 'UCPD_TypeDef',
    'USB': 'USB_TypeDef',
    'VREFBUF': 'VREFBUF_TypeDef',
    'WWDG': 'WWDG_TypeDef',
}

# Special cases where the peripheral name doesn't match the group pattern
SPECIAL_TYPES = {
    'ADC12_Common': 'ADC_Common_TypeDef',
    'ADC345_Common': 'ADC_Common_TypeDef',
    'LPUART1': 'USART_TypeDef',
    'UART4': 'USART_TypeDef',
    'UART5': 'USART_TypeDef',
}

# Aliases for common naming variations (SVD name -> common C name)
ALIASES = {
    'LPTIMER1': 'LPTIM1',
}

# DMA channel base addresses (STM32G4)
# Each DMA has 8 channels, each channel struct is 0x14 bytes (CCR, CNDTR, CPAR, CMAR, reserved)
DMA_CHANNELS = {
    'DMA1': (0x40020008, 8, 0x14),  # base address of channel 1, num channels, spacing
    'DMA2': (0x40020408, 8, 0x14),
}

# Peripherals to skip (not useful or no C type available)
SKIP_PERIPHERALS = {
    'MPU', 'NVIC', 'NVIC_STIR', 'SCB', 'SCB_ACTRL', 'STK', 'FPU', 'FPU_CPACR',
}


def parse_svd(svd_path):
    """Parse SVD file and extract peripheral names, addresses, and groups."""
    tree = ET.parse(svd_path)
    root = tree.getroot()

    peripherals = []
    for periph in root.findall('.//peripheral'):
        name = periph.find('name').text
        if name in SKIP_PERIPHERALS:
            continue

        # Handle derived peripherals
        derived_from = periph.get('derivedFrom')
        if derived_from:
            # Find the base peripheral's group
            for base_periph in root.findall('.//peripheral'):
                if base_periph.find('name').text == derived_from:
                    group_elem = base_periph.find('groupName')
                    group = group_elem.text if group_elem is not None else None
                    break
            else:
                group = None
        else:
            group_elem = periph.find('groupName')
            group = group_elem.text if group_elem is not None else None

        base_addr_elem = periph.find('baseAddress')
        if base_addr_elem is None:
            continue
        base_addr = int(base_addr_elem.text, 0)

        peripherals.append({
            'name': name,
            'group': group,
            'address': base_addr,
        })

    return peripherals


def get_c_type(periph, include_optional=False):
    """Determine the C typedef for a peripheral."""
    name = periph['name']
    group = periph['group']

    # Check special cases first
    if name in SPECIAL_TYPES:
        return SPECIAL_TYPES[name], False

    # Use group mapping (required types)
    if group and group in GROUP_TO_TYPE:
        return GROUP_TO_TYPE[group], False

    # Check optional types
    if include_optional and group and group in OPTIONAL_TYPES:
        return OPTIONAL_TYPES[group], True

    # Fallback: try to guess from name
    import re
    base_name = re.sub(r'\d+$', '', name)
    if base_name in GROUP_TO_TYPE:
        return GROUP_TO_TYPE[base_name], False
    if include_optional and base_name in OPTIONAL_TYPES:
        return OPTIONAL_TYPES[base_name], True

    return None, False


def generate_gdb_file(peripherals, output=sys.stdout, include_optional=False):
    """Generate GDB convenience variable definitions."""
    output.write("# Auto-generated GDB convenience variables for STM32G4 peripherals\n")
    output.write("# Generated by tools/generate_gdb_peripherals.py\n")
    output.write("#\n")
    output.write("# Usage: source this file in GDB, then use $PERIPH->REG syntax\n")
    output.write("# Example: print/x $TIM5->CR1\n")
    output.write("#\n\n")

    # Group by type for readability
    by_type = {}
    skipped = []
    for periph in peripherals:
        c_type, is_optional = get_c_type(periph, include_optional)
        if c_type is None:
            skipped.append(periph['name'])
            continue
        if c_type not in by_type:
            by_type[c_type] = []
        by_type[c_type].append(periph)

    # Output grouped by type
    for c_type in sorted(by_type.keys()):
        output.write(f"# {c_type}\n")
        for periph in sorted(by_type[c_type], key=lambda p: p['address']):
            name = periph['name']
            addr = periph['address']
            output.write(f"set ${name} = ({c_type}*){addr:#010x}\n")
        output.write("\n")

    # Add aliases
    if ALIASES:
        output.write("# Aliases (common naming variations)\n")
        for svd_name, alias in sorted(ALIASES.items()):
            output.write(f"set ${alias} = ${svd_name}\n")
        output.write("\n")

    # Add DMA channels
    output.write("# DMA_Channel_TypeDef (individual DMA channels)\n")
    for dma_name, (base_addr, num_channels, spacing) in sorted(DMA_CHANNELS.items()):
        for ch in range(1, num_channels + 1):
            ch_addr = base_addr + (ch - 1) * spacing
            output.write(f"set ${dma_name}_Channel{ch} = (DMA_Channel_TypeDef*){ch_addr:#010x}\n")
    output.write("\n")

    if skipped:
        output.write(f"# Skipped (no C type mapping): {', '.join(sorted(skipped))}\n")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <svd_file> [output_file]", file=sys.stderr)
        sys.exit(1)

    svd_path = Path(sys.argv[1])
    if not svd_path.exists():
        print(f"Error: SVD file not found: {svd_path}", file=sys.stderr)
        sys.exit(1)

    peripherals = parse_svd(svd_path)

    if len(sys.argv) >= 3:
        output_path = Path(sys.argv[2])
        with open(output_path, 'w') as f:
            generate_gdb_file(peripherals, f)
        print(f"Generated {output_path} with {len(peripherals)} peripherals")
    else:
        generate_gdb_file(peripherals)


if __name__ == '__main__':
    main()
