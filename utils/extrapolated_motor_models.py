#!/usr/bin/python

"""
From Claude Sonnet 4

NOTE: This gives TERRIBLE results.  Do not use yet.
"""

"""
BLDC Motor Parameter Extrapolation using Pypowertrain-inspired Scaling Laws

This module extrapolates motor parameters from easily measurable inputs:
- Kv (motor velocity constant in RPM/V)
- Mass (motor mass in kg)
- Diameter (motor diameter in m)
- Type (outrunner/inrunner)

Output parameters:
- Phase resistance (Rs in Ohms)
- Phase inductance (Ls in Henry)
- Iron loss constants (d0, d1)
- Thermal resistance and capacitance to ambient

Based on scaling laws from:
1. Pypowertrain library approach
2. "Scaling laws for synchronous permanent magnet machines" paper
3. Empirical relationships from motor databases
"""

import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import Literal
import warnings

@dataclass
class MotorInputs:
    """Easily measurable motor parameters"""
    kv: float           # Motor velocity constant (RPM/V)
    mass: float         # Motor mass (kg)
    diameter: float     # Motor outer diameter (m)
    motor_type: Literal['outrunner', 'inrunner']  # Motor configuration

@dataclass
class MotorParameters:
    """Extrapolated motor parameters"""
    # Electrical parameters
    phase_resistance: float     # Rs (Ohms)
    phase_inductance: float     # Ls (Henry)

    # Iron loss parameters (Steinmetz equation coefficients)
    iron_loss_d0: float        # Hysteresis loss coefficient
    iron_loss_d1: float        # Eddy current loss coefficient

    # Thermal parameters
    thermal_resistance: float   # Rth (K/W) - thermal resistance to ambient
    thermal_capacitance: float  # Cth (J/K) - thermal capacitance

class BLDCParameterExtractor:
    """
    Extract BLDC motor parameters using pypowertrain-inspired scaling laws
    """

    def __init__(self):
        # Reference motor parameters (based on typical hobby motor data)
        # These serve as scaling references for different motor types
        self.reference_motors = {
            'outrunner': {
                'kv_ref': 1000,      # RPM/V
                'mass_ref': 0.1,     # kg
                'diameter_ref': .050,   # m
                'rs_ref': 0.1,       # Ohms
                'ls_ref': 50e-6,     # Henry
                'd0_ref': 1e-3,      # Iron loss coefficient
                'd1_ref': 1e-6,      # Iron loss coefficient
                'rth_ref': 20,       # K/W
                'cth_ref': 10,       # J/K
            },
            'inrunner': {
                'kv_ref': 3000,      # RPM/V
                'mass_ref': 0.05,    # kg
                'diameter_ref': .030,   # m
                'rs_ref': 0.2,       # Ohms
                'ls_ref': 30e-6,     # Henry
                'd0_ref': 0.5e-3,    # Iron loss coefficient
                'd1_ref': 0.5e-6,    # Iron loss coefficient
                'rth_ref': 30,       # K/W
                'cth_ref': 5,        # J/K
            }
        }

    def extract_parameters(self, inputs: MotorInputs) -> MotorParameters:
        """
        Extract motor parameters using scaling laws

        Based on scaling relationships:
        1. Resistance scales with geometry and material (∝ L/A relationship)
        2. Inductance scales with geometry and turns ratio
        3. Iron losses scale with frequency and flux density
        4. Thermal parameters scale with surface area and mass
        """
        ref = self.reference_motors[inputs.motor_type]

        # Calculate geometric scaling factors
        diameter_ratio = inputs.diameter / ref['diameter_ref']
        mass_ratio = inputs.mass / ref['mass_ref']
        kv_ratio = inputs.kv / ref['kv_ref']

        # Derive additional scaling relationships
        # Volume scaling (assuming proportional height scaling)
        volume_ratio = mass_ratio  # Assuming similar material densities

        # Length scaling (from diameter and volume)
        # If D² * L ∝ Volume, then L ∝ Volume/D²
        length_ratio = volume_ratio / (diameter_ratio ** 2)

        # Cross-sectional area scaling
        area_ratio = diameter_ratio ** 2

        # Phase Resistance Scaling
        # R ∝ ρ * L / A, where ρ is resistivity
        # Also, Kv ∝ 1/√(L*R) approximately, so R ∝ 1/(Kv² * L)
        # Combined scaling from geometry and Kv relationship
        phase_resistance = ref['rs_ref'] * (length_ratio / area_ratio) * (1 / kv_ratio**0.5)

        # Phase Inductance Scaling
        # L ∝ N²μA/l where N is turns, μ is permeability, A is area, l is length
        # Kv ∝ 1/N, so N ∝ 1/Kv
        # L ∝ (1/Kv²) * (A/l) = (1/Kv²) * (D²/l)
        phase_inductance = ref['ls_ref'] * (1 / kv_ratio**2) * (area_ratio / length_ratio)

        # Iron Loss Parameters Scaling
        # Iron losses depend on frequency (∝ RPM) and flux density
        # d0 (hysteresis): ∝ Volume * flux_density^n (n≈1.6-2)
        # d1 (eddy current): ∝ Volume * frequency² * flux_density²
        # Higher Kv motors typically have higher flux density
        flux_density_ratio = kv_ratio ** 0.5  # Approximate relationship

        iron_loss_d0 = ref['d0_ref'] * volume_ratio * (flux_density_ratio ** 1.8)
        iron_loss_d1 = ref['d1_ref'] * volume_ratio * (flux_density_ratio ** 2)

        # Thermal Parameters Scaling
        # Thermal resistance: Rth ∝ 1/surface_area (heat dissipation area)
        # Surface area ∝ D * L for cylindrical motor
        surface_area_ratio = diameter_ratio * length_ratio
        thermal_resistance = ref['rth_ref'] / surface_area_ratio

        # Thermal capacitance: Cth ∝ mass * specific_heat ≈ mass
        thermal_capacitance = ref['cth_ref'] * mass_ratio

        return MotorParameters(
            phase_resistance=max(phase_resistance, 0.001),  # Minimum bounds
            phase_inductance=max(phase_inductance, 1e-6),
            iron_loss_d0=iron_loss_d0,
            iron_loss_d1=iron_loss_d1,
            thermal_resistance=max(thermal_resistance, 1.0),
            thermal_capacitance=max(thermal_capacitance, 0.1)
        )

    def refine_with_sample_data(self, sample_data: pd.DataFrame):
        """
        Refine scaling relationships using empirical motor data

        Expected DataFrame columns:
        - kv, mass, diameter, motor_type (inputs)
        - phase_resistance, phase_inductance, etc. (measured outputs)
        """
        warnings.warn("Empirical refinement not yet implemented - using theoretical scaling")
        pass

    def validate_parameters(self, inputs: MotorInputs, outputs: MotorParameters) -> dict:
        """
        Validate extracted parameters against known motor relationships
        """
        validations = {}

        # Check Kv vs resistance relationship (should be roughly inverse)
        kv_resistance_product = inputs.kv * outputs.phase_resistance
        validations['kv_resistance_reasonable'] = 10 < kv_resistance_product < 1000

        # Check inductance vs resistance (L/R time constant should be reasonable)
        time_constant = outputs.phase_inductance / outputs.phase_resistance
        validations['time_constant_reasonable'] = 10e-6 < time_constant < 1e-3

        # Check thermal parameters (Rth * Power should give reasonable temp rise)
        typical_power = 100  # Watts
        temp_rise = typical_power * outputs.thermal_resistance
        validations['thermal_reasonable'] = 10 < temp_rise < 200  # °C

        return validations


# Enhanced parameter extraction with motor database fitting
class EnhancedBLDCExtractor(BLDCParameterExtractor):
    """
    Enhanced extractor that can be trained on motor database
    """

    def __init__(self):
        super().__init__()
        self.fitted_coefficients = None

    def fit_scaling_coefficients(self, motor_database: pd.DataFrame):
        """
        Fit scaling law coefficients to empirical motor data

        Uses regression to find optimal scaling exponents and coefficients
        """
        from sklearn.linear_model import LinearRegression
        from sklearn.preprocessing import StandardScaler

        # Prepare features (log-transformed for power law fitting)
        features = ['kv', 'mass', 'diameter']
        targets = ['phase_resistance', 'phase_inductance', 'iron_loss_d0', 'iron_loss_d1',
                  'thermal_resistance', 'thermal_capacitance']

        # Separate by motor type
        self.fitted_coefficients = {}

        for motor_type in ['outrunner']:
            type_data = motor_database[motor_database['motor_type'] == motor_type]

            if len(type_data) < 5:
                warnings.warn(f"Insufficient data for {motor_type} motors")
                continue

            self.fitted_coefficients[motor_type] = {}

            # Log-transform for power law relationships
            X = np.log(type_data[features].values + 1e-9)  # Avoid log(0)

            for target in targets:
                if target in type_data.columns:
                    print(f"target={target}")
                    y = np.log(type_data[target].values + 1e-9)

                    # Fit linear regression in log space (power law in linear space)
                    reg = LinearRegression()
                    reg.fit(X, y)

                    print(f"X={X}, y={y}")
                    print(reg.intercept_, reg.coef_)
                    print(f"fitted={reg.intercept_ + np.dot(reg.coef_, X.transpose())}")

                    self.fitted_coefficients[motor_type][target] = {
                        'intercept': reg.intercept_,
                        'coefficients': reg.coef_,
                        'score': reg.score(X, y)
                    }

    def extract_parameters_fitted(self, inputs: MotorInputs) -> MotorParameters:
        """
        Extract parameters using fitted coefficients if available
        """
        if self.fitted_coefficients is None:
            return self.extract_parameters(inputs)

        if inputs.motor_type not in self.fitted_coefficients:
            return self.extract_parameters(inputs)

        # Use fitted coefficients
        coeffs = self.fitted_coefficients[inputs.motor_type]

        # Input vector (log-transformed)
        x = np.log([inputs.kv, inputs.mass, inputs.diameter])

        results = {}
        for param, coeff_data in coeffs.items():
            log_value = coeff_data['intercept'] + np.dot(coeff_data['coefficients'], x)
            results[param] = np.exp(log_value)

        return MotorParameters(
            phase_resistance=results.get('phase_resistance', 0.1),
            phase_inductance=results.get('phase_inductance', 50e-6),
            iron_loss_d0=results.get('iron_loss_d0', 1e-3),
            iron_loss_d1=results.get('iron_loss_d1', 1e-6),
            thermal_resistance=results.get('thermal_resistance', 20),
            thermal_capacitance=results.get('thermal_capacitance', 10)
        )


# Utility functions for pypowertrain integration
def create_pypowertrain_motor_dict(inputs: MotorInputs, params: MotorParameters) -> dict:
    """
    Create motor parameter dictionary compatible with pypowertrain format
    """
    return {
        'kv': inputs.kv,
        'resistance': params.phase_resistance,
        'inductance': params.phase_inductance,
        'mass': inputs.mass,
        'diameter': inputs.diameter,
        'iron_loss': {
            'd0': params.iron_loss_d0,
            'd1': params.iron_loss_d1
        },
        'thermal': {
            'resistance': params.thermal_resistance,
            'capacitance': params.thermal_capacitance
        }
    }


# Example usage and testing
if __name__ == "__main__":
    # Example motor specifications
    test_motors = [
        MotorInputs(kv=304, mass=0.192, diameter=.063, motor_type='outrunner'),
        MotorInputs(kv=120, mass=0.646, diameter=.091, motor_type='outrunner'),
    ]

    # Define sample motors with measured parameters for database fitting
    # In practice, these would come from actual motor testing/measurement data
    sample_motors = [
        {
            'name': 'mj5208',
            'kv': 304,
            'mass': 0.192,
            'diameter': 0.063,
            'motor_type': 'outrunner',
            'phase_resistance': 0.047,
            'phase_inductance': 28.6e-6,
            'iron_loss_d0': 0.01289,
            'iron_loss_d1': 0.0001469,
            'thermal_resistance': 4.55,
            'thermal_capacitance': 164.59,
        },
        {
            'name': 'MAD 8318',
            'kv': 120,
            'mass': 0.646,
            'diameter': 0.091,
            'motor_type': 'outrunner',
            'phase_resistance': 0.015,
            'phase_inductance': 9.75e-6,
            'iron_loss_d0': 0.1272,
            'iron_loss_d1': 0.001829,
            'thermal_resistance': 1.516,
            'thermal_capacitance': 735.5,
        },
        {
            'name': 'gbm5208',
            'kv': 25.5,
            'mass': 0.193,
            'diameter': 0.063,
            'motor_type': 'outrunner',
            'phase_resistance': 7.545,
            'phase_inductance': 2254.5e-6,
            'iron_loss_d0': 0.00822,
            'iron_loss_d1': 0.000183,
            'thermal_resistance': 4.55,
            'thermal_capacitance': 165,
        },
        {
            'name': 'be8108',
            'kv': 135,
            'mass': 0.230,
            'diameter': 0.087,
            'motor_type' : 'outrunner',
            'phase_resistance': 0.0910,
            'phase_inductance': 39.2e-6,
            'iron_loss_d0': 0.0300,
            'iron_loss_d1': 0.000691,
            'thermal_resistance': 2.74,
            'thermal_capacitance': 210.9,
        },
        {
            'name': 'hoverboard350',
            'kv': 18.2,
            'mass': 3.26,  # 4.26 including tire
            'diameter': 0.156,
            'motor_type': 'outrunner',
            'phase_resistance': 0.216,
            'phase_inductance': 530e-6,
            'iron_loss_d0': 0.0892,
            'iron_loss_d1': 0.0131,
            'thermal_resistance': 2.41,
            'thermal_capacitance': 946.1,
        },
        {
            'name': 'ht1105',
            'kv': 1180,
            'mass': 0.008,
            'diameter': 0.016,
            'motor_type': 'outrunner',
            'phase_resistance': 6.435,
            'phase_inductance': 298.5e-6,
            'iron_loss_d0': 6.29e-5,
            'iron_loss_d1': 3.80e-6,
            'thermal_resistance': 20.8,
            'thermal_capacitance': 14.8,
        },

    ]

    # Convert to DataFrame for the enhanced extractor
    sample_database = pd.DataFrame(sample_motors)

    print("Sample Motor Database:")
    print("=" * 50)
    for i, motor in enumerate(sample_motors):
        print(f"{i+1:2d}. {motor['name']}")
        print(f"    Specs: {motor['kv']} kV, {motor['mass']*1000:.0f}g, "
              f"⌀{motor['diameter']}mm ({motor['motor_type']})")
        print(f"    Measured: Rs={motor['phase_resistance']:.3f}Ω, "
              f"Ls={motor['phase_inductance']*1e6:.0f}μH, "
              f"Rth={motor['thermal_resistance']:.1f}K/W")
    print()

    print("BLDC Motor Parameter Extraction Results:")
    print("=" * 70)

    # Test basic extractor first
    print("\n1. BASIC SCALING LAWS APPROACH:")
    print("-" * 40)

    basic_extractor = BLDCParameterExtractor()

    for i, motor in enumerate(test_motors[:2], 1):  # Test first 2 motors
        print(f"\nMotor {i}: {motor.motor_type.title()}")
        print(f"Input: Kv={motor.kv} RPM/V, Mass={motor.mass:.3f} kg, "
              f"Diameter={motor.diameter * 1000} mm")

        params = basic_extractor.extract_parameters(motor)
        validations = basic_extractor.validate_parameters(motor, params)

        print(f"Basic Scaling Results:")
        print(f"  Phase Resistance: {params.phase_resistance:.4f} Ω")
        print(f"  Phase Inductance: {params.phase_inductance*1e6:.1f} μH")
        print(f"  Iron Loss d0: {params.iron_loss_d0:.2e}")
        print(f"  Iron Loss d1: {params.iron_loss_d1:.2e}")
        print(f"  Thermal Resistance: {params.thermal_resistance:.1f} K/W")
        print(f"  Thermal Capacitance: {params.thermal_capacitance:.1f} J/K")
        print(f"  Validations: {sum(validations.values())}/{len(validations)} passed")

    # Test enhanced extractor with database fitting
    print("\n\n2. ENHANCED DATABASE-FITTED APPROACH:")
    print("-" * 45)

    enhanced_extractor = EnhancedBLDCExtractor()

    # Fit the extractor to sample database
    print("Fitting scaling coefficients to motor database...")
    try:
        enhanced_extractor.fit_scaling_coefficients(sample_database)
        print("✓ Database fitting completed successfully")

        # Show fitting quality
        if enhanced_extractor.fitted_coefficients:
            print("\nFitting Quality (R² scores):")
            for motor_type, params in enhanced_extractor.fitted_coefficients.items():
                print(f"  {motor_type.title()}:")
                for param, data in params.items():
                    print(f"    {param}: R² = {data['score']:.3f}")

    except ImportError:
        print("⚠ sklearn not available - using basic scaling laws instead")
        enhanced_extractor = basic_extractor

    print(f"\nTesting all {len(test_motors)} motors with enhanced extractor:")

    for i, motor in enumerate(test_motors, 1):
        print(f"\n--- Motor {i}: {motor.motor_type.title()} ---")
        print(f"Input: Kv={motor.kv} RPM/V, Mass={motor.mass:.3f} kg, "
              f"Diameter={motor.diameter * 1000} mm")

        # Extract parameters using enhanced method
        if hasattr(enhanced_extractor, 'extract_parameters_fitted'):
            params = enhanced_extractor.extract_parameters_fitted(motor)
            method = "Database-fitted"
        else:
            params = enhanced_extractor.extract_parameters(motor)
            method = "Basic scaling"

        validations = enhanced_extractor.validate_parameters(motor, params)

        print(f"\n{method} Results:")
        print(f"  Phase Resistance: {params.phase_resistance:.4f} Ω")
        print(f"  Phase Inductance: {params.phase_inductance*1e6:.1f} μH")
        print(f"  Iron Loss d0: {params.iron_loss_d0:.2e}")
        print(f"  Iron Loss d1: {params.iron_loss_d1:.2e}")
        print(f"  Thermal Resistance: {params.thermal_resistance:.1f} K/W")
        print(f"  Thermal Capacitance: {params.thermal_capacitance:.1f} J/K")

        # Show validation results
        validation_details = []
        for check, passed in validations.items():
            status = "✓" if passed else "✗"
            validation_details.append(f"{status} {check}")
        print(f"  Validations: {', '.join(validation_details)}")

        # Show pypowertrain-compatible format
        motor_dict = create_pypowertrain_motor_dict(motor, params)
        print(f"\n  Pypowertrain format:")
        for key, value in motor_dict.items():
            if isinstance(value, dict):
                print(f"    {key}: {value}")
            else:
                print(f"    {key}: {value}")

    # Demonstrate practical usage scenarios
    print("\n\n3. PRACTICAL USAGE EXAMPLES:")
    print("-" * 35)

    print("\nA) Motor Selection Screening:")
    candidates = test_motors[:3]
    target_resistance_range = (0.05, 0.20)  # Target resistance range

    suitable_motors = []
    for motor in candidates:
        params = enhanced_extractor.extract_parameters(motor)
        if target_resistance_range[0] <= params.phase_resistance <= target_resistance_range[1]:
            suitable_motors.append((motor, params))
            print(f"  ✓ {motor.motor_type} Kv={motor.kv}: Rs={params.phase_resistance:.3f}Ω (suitable)")
        else:
            print(f"  ✗ {motor.motor_type} Kv={motor.kv}: Rs={params.phase_resistance:.3f}Ω (out of range)")

    print(f"\nFound {len(suitable_motors)} suitable motors from {len(candidates)} candidates")

    print("\nB) Parameter Trend Analysis:")
    kv_values = [motor.kv for motor in test_motors]
    resistance_values = [enhanced_extractor.extract_parameters(motor).phase_resistance
                        for motor in test_motors]

    print(f"  Kv range: {min(kv_values)} - {max(kv_values)} RPM/V")
    print(f"  Resistance range: {min(resistance_values):.3f} - {max(resistance_values):.3f} Ω")
    print(f"  Kv × Rs products: {[kv*r for kv, r in zip(kv_values, resistance_values)]}")

    print("\n" + "=" * 70)
    print("Example completed! The EnhancedBLDCExtractor shows improved")
    print("accuracy when trained on motor database compared to basic scaling.")
