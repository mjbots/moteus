#!/usr/bin/env node

// Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// CLI tool for moteus Performance Analysis Tool
// Uses the core computation logic extracted from docs/mpat.html

import {
    fieldDefinitions,
    Motor,
    OperatingPoint,
    evaluateMaxCurrent,
    evaluateOperatingPoint,
    evaluateMaxTorque,
    evaluateMaxVelocity,
    evaluateMinMovementTime,
} from './mpat_core.mjs';

// Build registries from fieldDefinitions
function buildRegistry(fieldName) {
    const fieldDef = fieldDefinitions[fieldName];
    if (!fieldDef || fieldDef.type !== 'constrained' || !fieldDef.options) {
        return {};
    }
    const registry = {};
    for (const [name, optionDef] of Object.entries(fieldDef.options)) {
        if (optionDef.model) {
            registry[name] = optionDef.model;
        }
    }
    return registry;
}

// Get valid options for a constrained field
function getOptions(fieldName) {
    const fieldDef = fieldDefinitions[fieldName];
    if (!fieldDef || fieldDef.type !== 'constrained' || !fieldDef.options) {
        return [];
    }
    return Object.keys(fieldDef.options);
}

// Build registries from the source of truth
const controllers = buildRegistry('controller');
const motors = buildRegistry('motor');

// Analysis functions registry
const analyses = {
    'max_current': evaluateMaxCurrent,
    'max_torque': evaluateMaxTorque,
    'max_velocity': evaluateMaxVelocity,
    'operating_point': evaluateOperatingPoint,
    'move_time': evaluateMinMovementTime,
};

function parseArgs(args) {
    const result = {
        help: false,
        analysis: 'max_torque',
        controller: 'moteus-x1',
        controller_cooling: 'none',
        motor: 'mad8318',
        motor_cooling: 'none',
        voltage: 24,
        velocity: 0.0,
        torque: null,
        time: Infinity,
        gear_reduction: 1.0,
        ambient_temp: 25,
        max_controller_temp: null,  // Will use controller default
        max_motor_temp: null,       // Will use motor default
        pwm: null,                  // Will use controller default
        move_distance: null,
        load_inertia: 0.0,
        output: 'torque',
        json: false,
        // Custom motor parameters
        'motor.kv': null,
        'motor.r': null,
        'motor.l': null,
        'motor.thermal_r': null,
        'motor.thermal_c': null,
        'motor.d0': 1e-12,
        'motor.d1': 1e-12,
        'motor.inertia': 0,
    };

    for (let i = 0; i < args.length; i++) {
        const arg = args[i];
        if (arg === '--help' || arg === '-h') {
            result.help = true;
        } else if (arg === '--json') {
            result.json = true;
        } else if (arg.startsWith('--')) {
            const key = arg.slice(2);
            const value = args[++i];
            if (value === undefined) {
                console.error(`Error: Missing value for ${arg}`);
                process.exit(1);
            }
            // Handle numeric conversions
            if (['voltage', 'velocity', 'torque', 'time', 'gear_reduction',
                 'ambient_temp', 'max_controller_temp', 'max_motor_temp', 'pwm',
                 'move_distance', 'load_inertia',
                 'motor.kv', 'motor.r', 'motor.l', 'motor.thermal_r', 'motor.thermal_c',
                 'motor.d0', 'motor.d1', 'motor.inertia'].includes(key)) {
                if (value.toLowerCase() === 'infinity') {
                    result[key] = Infinity;
                } else {
                    result[key] = parseFloat(value);
                }
            } else {
                result[key] = value;
            }
        }
    }

    return result;
}

function printHelp() {
    // Build help text dynamically from fieldDefinitions
    const controllerOptions = getOptions('controller');
    const motorOptions = getOptions('motor');
    const analysisOptions = getOptions('analysis');
    const outputOptions = getOptions('output');
    const controllerCoolingOptions = getOptions('controller_cooling');
    const motorCoolingOptions = getOptions('motor_cooling');

    console.log(`
moteus Performance Analysis Tool - CLI

Usage: mpat_cli.mjs [options]

Analysis Types (--analysis):
${analysisOptions.map(name => `  ${name.padEnd(18)} ${fieldDefinitions.analysis.options[name].help}`).join('\n')}

Controllers (--controller):
${controllerOptions.map(name => `  ${name}`).join('\n')}

Motors (--motor):
${motorOptions.map(name => `  ${name}`).join('\n')}

Controller Cooling (--controller_cooling):
${controllerCoolingOptions.map(name => `  ${name.padEnd(18)} ${fieldDefinitions.controller_cooling.options[name].help}`).join('\n')}

Motor Cooling (--motor_cooling):
${motorCoolingOptions.map(name => `  ${name.padEnd(18)} ${fieldDefinitions.motor_cooling.options[name].help}`).join('\n')}

Output Fields (--output):
${outputOptions.map(name => `  ${name.padEnd(18)} ${fieldDefinitions.output.options[name].help}`).join('\n')}

Required Options:
  --analysis TYPE   Analysis type (see above)
  --controller NAME Controller type (see above)
  --voltage V       Supply voltage in volts

Analysis-Specific Options:
  --motor NAME      Motor type (required for most analyses)
  --velocity V      Velocity in Hz (required for max_torque, operating_point)
  --torque T        Torque in Nm (required for max_velocity, operating_point)
  --move_distance D Distance in revolutions (required for move_time)
  --load_inertia I  Load inertia in kg*m^2 (required for move_time)

Optional Parameters:
  --time T          Time duration in seconds (default: Infinity for steady-state)
  --gear_reduction R Gear reduction ratio (default: 1.0)
  --ambient_temp T  Ambient temperature in C (default: 25)
  --controller_cooling TYPE  Cooling type (default: none)
  --motor_cooling TYPE       Cooling type (default: none)
  --pwm P           PWM frequency in Hz (default: controller default)
  --max_controller_temp T    Max controller temp C (default: controller limit)
  --max_motor_temp T         Max motor temp C (default: motor limit)
  --output FIELD    Specific output field to display

Custom Motor Parameters (when --motor model):
  --motor.kv KV           Motor Kv rating (RPM/V)
  --motor.r R             Phase-to-phase resistance in ohms
  --motor.l L             Phase-to-phase inductance in henries
  --motor.thermal_r R     Thermal resistance in C/W
  --motor.thermal_c C     Thermal capacitance in J/C
  --motor.d0 D0           Linear drag coefficient (default: 1e-12)
  --motor.d1 D1           Quadratic drag coefficient (default: 1e-12)
  --motor.inertia I       Rotor inertia in kg*m^2 (default: 0)

Output Options:
  --json            Output results as JSON
  --help, -h        Show this help message

Examples:
  # Maximum torque at 10 Hz with moteus-x1 and mj5208 at 48V
  ./mpat.py --analysis max_torque --controller moteus-x1 --motor mj5208 \\
            --voltage 48 --velocity 10

  # Operating point analysis
  ./mpat.py --analysis operating_point --controller moteus-n1 --motor mad8318 \\
            --voltage 24 --torque 0.5 --velocity 5

  # Custom motor model
  ./mpat.py --analysis max_torque --controller moteus-x1 --motor model \\
            --motor.kv 200 --motor.r 0.05 --motor.l 0.00001 \\
            --motor.thermal_r 1.0 --motor.thermal_c 100 \\
            --voltage 48 --velocity 10
`);
}

function formatOperatingPoint(op) {
    if (op === null) {
        return 'Analysis returned null (constraints not satisfiable)';
    }

    const fieldInfo = OperatingPoint.getFieldInfo();
    const lines = [];

    if (op._infeasible_reason) {
        lines.push(`Warning: ${op._infeasible_reason}`);
    }
    if (op._limiting_factor) {
        lines.push(`Limiting factor: ${op._limiting_factor}`);
    }

    lines.push('');
    lines.push('Results:');

    for (const [field, info] of Object.entries(fieldInfo)) {
        const value = op[field];
        if (value !== null && value !== undefined) {
            lines.push(`  ${info.displayName}: ${value.toFixed(info.precision)} ${info.unit}`);
        }
    }

    return lines.join('\n');
}

function main() {
    const args = parseArgs(process.argv.slice(2));

    if (args.help || process.argv.length <= 2) {
        printHelp();
        process.exit(0);
    }

    // Validate required options
    if (!args.analysis) {
        console.error('Error: --analysis is required');
        process.exit(1);
    }
    if (!analyses[args.analysis]) {
        console.error(`Error: Unknown analysis type: ${args.analysis}`);
        console.error(`Valid types: ${Object.keys(analyses).join(', ')}`);
        process.exit(1);
    }
    if (!args.controller) {
        console.error('Error: --controller is required');
        process.exit(1);
    }
    if (!controllers[args.controller]) {
        console.error(`Error: Unknown controller: ${args.controller}`);
        console.error(`Valid controllers: ${Object.keys(controllers).join(', ')}`);
        process.exit(1);
    }
    if (args.voltage === null) {
        console.error('Error: --voltage is required');
        process.exit(1);
    }

    // Build configuration
    const controller = controllers[args.controller]();
    let motor = null;

    if (args.motor === 'model') {
        // Custom motor model
        const required = ['motor.kv', 'motor.r', 'motor.l', 'motor.thermal_r', 'motor.thermal_c'];
        for (const field of required) {
            if (args[field] === null) {
                console.error(`Error: --${field} is required for custom motor model`);
                process.exit(1);
            }
        }
        motor = new Motor({
            name: 'model',
            displayName: 'Custom Model',
            maxTemp: 80,
            Kv: args['motor.kv'],
            R_pp: args['motor.r'],
            L_pp: args['motor.l'],
            d0: args['motor.d0'],
            d1: args['motor.d1'],
            inertia: args['motor.inertia'],
            thermal: {
                'none': {
                    R: args['motor.thermal_r'],
                    C: args['motor.thermal_c']
                }
            }
        });
    } else if (args.motor && motors[args.motor]) {
        motor = motors[args.motor]();
    } else if (args.motor) {
        console.error(`Error: Unknown motor: ${args.motor}`);
        console.error(`Valid motors: ${Object.keys(motors).join(', ')}, model`);
        process.exit(1);
    }

    const config = {
        controller: controller,
        controller_cooling: args.controller_cooling,
        motor: motor,
        motor_cooling: args.motor_cooling,
        voltage: args.voltage,
        velocity: args.velocity,
        torque: args.torque,
        time: args.time,
        gear_reduction: args.gear_reduction,
        ambient_temp: args.ambient_temp,
        max_controller_temp: args.max_controller_temp ?? controller.getMaxTemp(),
        max_motor_temp: args.max_motor_temp ?? (motor?.getMaxTemp() ?? 80),
        pwm: args.pwm ?? controller.getDefaultPwm(),
        move_distance: args.move_distance,
        load_inertia: args.load_inertia,
    };

    // Run analysis
    const analysisFunc = analyses[args.analysis];
    const result = analysisFunc(config);

    // Output results
    if (args.json) {
        const output = result ? { ...result } : null;
        // Remove non-serializable fields
        if (output) {
            delete output._infeasible_reason;
            delete output._limiting_factor;
        }
        console.log(JSON.stringify(output, null, 2));
    } else if (args.output && result) {
        const value = result[args.output];
        if (value !== null && value !== undefined) {
            console.log(OperatingPoint.formatValue(args.output, value));
        } else {
            console.log('N/A');
        }
    } else {
        console.log(formatOperatingPoint(result));
    }
}

main();
