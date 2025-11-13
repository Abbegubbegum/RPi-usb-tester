#!/usr/bin/env python3
"""
Plot ADC samples from USB power test report.
Converts raw ADC codes to voltage (mV) and generates time-series plots.
"""
import json
import argparse
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


# ADC and voltage divider constants (must match C code)
VREF_MV = 3300
ADC_MAX = 4095
VBUS_DIV_R1 = 675  # ohms
VBUS_DIV_R2 = 990  # ohms

# Timing constants
ADC_SAMPLE_RATE_HZ = 80000  # 80 kS/s
ADC_SAMPLES_PER_WINDOW = 9600
TRANSIENT_MS = 20
STEP_WINDOW_MS = 120


def adc_code_to_mv(code):
    """Convert ADC code to millivolts at ADC input"""
    return int(VREF_MV * code / ADC_MAX)


def convert_mv_res_divider(mv):
    """Convert voltage after resistor divider to actual VBUS voltage"""
    divider_scale = (VBUS_DIV_R1 + VBUS_DIV_R2) / VBUS_DIV_R2
    return int(mv * divider_scale)


def adc_code_to_vbus_mv(code):
    """Convert ADC code directly to VBUS voltage in mV"""
    return convert_mv_res_divider(adc_code_to_mv(code))


def plot_port_adc_samples(port_data, output_dir=None):
    """
    Plot ADC samples for a single port.

    Args:
        port_data: Dictionary containing port test results
        output_dir: Optional directory to save plots
    """
    port = port_data.get("port", "unknown")
    power_report = port_data.get("power_report")

    if not power_report:
        print(f"Port {port}: No power report data available")
        return

    # Check for new format (separate idle and step samples)
    adc_samples_idle = power_report.get("adc_samples_idle")
    adc_samples_steps = power_report.get("adc_samples_steps")

    # Fall back to old format if new format not available
    if adc_samples_idle is None and adc_samples_steps is None:
        adc_samples = power_report.get("adc_samples")
        if not adc_samples:
            print(f"Port {port}: No ADC samples available")
            return
        # Use old single-buffer plotting
        plot_single_buffer(port_data, adc_samples, output_dir)
        return

    if not adc_samples_idle or not adc_samples_steps:
        print(f"Port {port}: Incomplete ADC sample data")
        return

    # Plot idle + all load steps
    plot_multi_step_samples(port_data, adc_samples_idle, adc_samples_steps, output_dir)


def plot_single_buffer(port_data, adc_samples, output_dir=None):
    """Plot ADC samples from old single-buffer format"""
    port = port_data.get("port", "unknown")
    power_report = port_data.get("power_report")

    # Convert ADC codes to voltage
    voltages_mv = [adc_code_to_vbus_mv(code) for code in adc_samples]
    voltages_v = [v / 1000.0 for v in voltages_mv]

    # Create time axis in milliseconds
    time_ms = np.linspace(0, STEP_WINDOW_MS, len(adc_samples))

    # Get power report statistics for reference
    v_idle_mv = power_report.get("v_idle_mV", 0)
    v_mean_list = power_report.get("v_mean_mV", [])
    v_min_list = power_report.get("v_min_mV", [])
    loads_ma = power_report.get("loads_mA", [])

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle(f'Port {port} - VBUS Voltage During Power Test', fontsize=14, fontweight='bold')

    # Plot 1: Full voltage trace
    ax1.plot(time_ms, voltages_v, linewidth=0.5, color='blue', alpha=0.7)
    ax1.set_xlabel('Time (ms)', fontsize=10)
    ax1.set_ylabel('VBUS Voltage (V)', fontsize=10)
    ax1.set_title('Complete ADC Sample Trace', fontsize=11)
    ax1.grid(True, alpha=0.3)

    # Add idle voltage reference line
    if v_idle_mv > 0:
        ax1.axhline(y=v_idle_mv/1000.0, color='green', linestyle='--',
                   linewidth=1, label=f'Idle: {v_idle_mv/1000:.3f}V', alpha=0.7)

    # Add mean voltages for each load step if available
    if v_mean_list and loads_ma:
        for i, (v_mean, load) in enumerate(zip(v_mean_list, loads_ma)):
            if v_mean > 0:
                ax1.axhline(y=v_mean/1000.0, color='red', linestyle=':',
                           linewidth=0.8, alpha=0.5)

    ax1.legend(loc='best', fontsize=9)

    # Plot 2: Zoomed view of transient region (first 20ms)
    transient_samples = int((TRANSIENT_MS / STEP_WINDOW_MS) * len(adc_samples))
    time_transient = time_ms[:transient_samples]
    voltages_transient = voltages_v[:transient_samples]

    ax2.plot(time_transient, voltages_transient, linewidth=1, color='red', alpha=0.8)
    ax2.set_xlabel('Time (ms)', fontsize=10)
    ax2.set_ylabel('VBUS Voltage (V)', fontsize=10)
    ax2.set_title(f'Transient Region (First {TRANSIENT_MS}ms) - Load Step Response', fontsize=11)
    ax2.grid(True, alpha=0.3)

    # Add min voltage marker if available
    if v_min_list:
        min_voltage = min([v for v in v_min_list if v > 0], default=0)
        if min_voltage > 0:
            ax2.axhline(y=min_voltage/1000.0, color='orange', linestyle='--',
                       linewidth=1, label=f'Min: {min_voltage/1000:.3f}V', alpha=0.7)
            ax2.legend(loc='best', fontsize=9)

    # Add statistics text box
    stats_text = f"Sample Rate: {ADC_SAMPLE_RATE_HZ/1000:.0f} kS/s\n"
    stats_text += f"Samples: {len(adc_samples)}\n"
    stats_text += f"Duration: {STEP_WINDOW_MS} ms\n"
    if v_idle_mv > 0:
        stats_text += f"Idle: {v_idle_mv/1000:.3f} V\n"
    if v_mean_list:
        stats_text += f"Loads: {', '.join([str(l) for l in loads_ma])} mA"

    fig.text(0.02, 0.02, stats_text, fontsize=8, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

    plt.tight_layout(rect=[0, 0.05, 1, 0.97])

    # Save or show plot
    if output_dir:
        output_path = Path(output_dir) / f"port_{port}_adc_samples.png"
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot: {output_path}")
        plt.close()
    else:
        plt.show()


def plot_multi_step_samples(port_data, adc_samples_idle, adc_samples_steps, output_dir=None):
    """
    Plot ADC samples for each load step as separate figures.

    Args:
        port_data: Dictionary containing port test results
        adc_samples_idle: ADC samples for idle state
        adc_samples_steps: List of ADC sample arrays for each load step
        output_dir: Optional directory to save plots
    """
    port = port_data.get("port", "unknown")
    power_report = port_data.get("power_report")

    # Get power report statistics
    v_idle_mv = power_report.get("v_idle_mV", 0)
    v_mean_list = power_report.get("v_mean_mV", [])
    v_min_list = power_report.get("v_min_mV", [])
    v_max_list = power_report.get("v_max_mV", [])
    loads_ma = power_report.get("loads_mA", [])

    # Plot each load step as a separate figure
    colors = ['blue', 'red', 'orange', 'purple', 'brown']
    for i, samples in enumerate(adc_samples_steps):
        fig, ax = plt.subplots(figsize=(14, 6))

        voltages = [adc_code_to_vbus_mv(code) / 1000.0 for code in samples]
        time_ms = np.linspace(0, STEP_WINDOW_MS, len(samples))

        color = colors[i % len(colors)]

        ax.plot(time_ms, voltages, linewidth=0.7, color=color, alpha=0.7)
        ax.set_xlabel('Time (ms)', fontsize=11)
        ax.set_ylabel('VBUS (V)', fontsize=11)

        # Build title with statistics
        title = f'Port {port} - Load Step {i+1}'
        if i < len(loads_ma):
            title += f' ({loads_ma[i]} mA)'
        if i < len(v_mean_list) and v_mean_list[i] > 0:
            title += f' - Avg: {v_mean_list[i]/1000:.3f}V'
        title += f' | Idle: {v_idle_mv/1000:.3f}V'

        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)

        # Add reference lines
        show_legend = False
        if i < len(v_mean_list) and v_mean_list[i] > 0:
            ax.axhline(y=v_mean_list[i]/1000.0, color=color, linestyle='--',
                      linewidth=1, alpha=0.5, label=f'Mean: {v_mean_list[i]/1000:.3f}V')
            show_legend = True
        if i < len(v_min_list) and v_min_list[i] > 0:
            ax.axhline(y=v_min_list[i]/1000.0, color='red', linestyle=':',
                      linewidth=1, alpha=0.5, label=f'Min: {v_min_list[i]/1000:.3f}V')
            show_legend = True
        if i < len(v_max_list) and v_max_list[i] > 0:
            ax.axhline(y=v_max_list[i]/1000.0, color='green', linestyle=':',
                      linewidth=1, alpha=0.5, label=f'Max: {v_max_list[i]/1000:.3f}V')
            show_legend = True

        if show_legend:
            ax.legend(loc='upper right', fontsize=9)

        plt.tight_layout()

        # Save or show plot
        if output_dir:
            output_path = Path(output_dir) / f"port_{port}_step_{i+1}_{loads_ma[i] if i < len(loads_ma) else 'unknown'}mA.png"
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"Saved step {i+1} plot: {output_path}")
            plt.close()
        else:
            plt.show()

    # Also create overlay plots
    plot_all_steps_overlay(port_data, adc_samples_idle, adc_samples_steps, output_dir)
    plot_transient_comparison(port_data, adc_samples_idle, adc_samples_steps, output_dir)


def plot_all_steps_overlay(port_data, adc_samples_idle, adc_samples_steps, output_dir=None):
    """Plot overlay of all load steps on the same graph (full 120ms window)"""
    port = port_data.get("port", "unknown")
    power_report = port_data.get("power_report")
    loads_ma = power_report.get("loads_mA", [])
    v_idle_mv = power_report.get("v_idle_mV", 0)

    fig, ax = plt.subplots(figsize=(14, 8))
    fig.suptitle(f'Port {port} - All Load Steps Overlay (Full {STEP_WINDOW_MS}ms)',
                fontsize=14, fontweight='bold')

    colors = ['blue', 'red', 'orange', 'purple', 'brown']

    # Plot each load step
    for i, samples in enumerate(adc_samples_steps):
        voltages = [adc_code_to_vbus_mv(code) / 1000.0 for code in samples]
        time_ms = np.linspace(0, STEP_WINDOW_MS, len(samples))
        color = colors[i % len(colors)]

        label = f'Step {i+1}'
        if i < len(loads_ma):
            label += f' ({loads_ma[i]} mA)'

        ax.plot(time_ms, voltages, linewidth=1.0, color=color, alpha=0.7, label=label)

    # Add idle voltage reference line
    if v_idle_mv > 0:
        ax.axhline(y=v_idle_mv/1000.0, color='green', linestyle='--',
                  linewidth=1.5, alpha=0.6, label=f'Idle: {v_idle_mv/1000:.3f}V')

    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('VBUS Voltage (V)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    if output_dir:
        output_path = Path(output_dir) / f"port_{port}_all_steps_overlay.png"
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved all steps overlay: {output_path}")
        plt.close()
    else:
        plt.show()


def plot_transient_comparison(port_data, adc_samples_idle, adc_samples_steps, output_dir=None):
    """Plot overlay of transient regions for all load steps"""
    port = port_data.get("port", "unknown")
    power_report = port_data.get("power_report")
    loads_ma = power_report.get("loads_mA", [])

    fig, ax = plt.subplots(figsize=(12, 8))
    fig.suptitle(f'Port {port} - Load Step Transient Response (First {TRANSIENT_MS}ms)',
                fontsize=14, fontweight='bold')

    transient_samples = int((TRANSIENT_MS / STEP_WINDOW_MS) * len(adc_samples_idle))
    colors = ['blue', 'red', 'orange', 'purple', 'brown']

    # Plot each load step transient (skip idle - it only shows power-up/cap charging)
    for i, samples in enumerate(adc_samples_steps):
        voltages = [adc_code_to_vbus_mv(code) / 1000.0 for code in samples[:transient_samples]]
        time_ms = np.linspace(0, TRANSIENT_MS, len(voltages))
        color = colors[i % len(colors)]

        label = f'Step {i+1}'
        if i < len(loads_ma):
            label += f' ({loads_ma[i]} mA)'

        ax.plot(time_ms, voltages, linewidth=1.5, color=color, alpha=0.8, label=label)

    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('VBUS Voltage (V)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    if output_dir:
        output_path = Path(output_dir) / f"port_{port}_transient_comparison.png"
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved transient comparison: {output_path}")
        plt.close()
    else:
        plt.show()


def plot_all_ports_overlay(report_data, output_dir=None):
    """
    Create an overlay plot of all ports on the same axes.

    Args:
        report_data: Full test report dictionary
        output_dir: Optional directory to save plots
    """
    fig, ax = plt.subplots(figsize=(14, 8))
    fig.suptitle('All Ports - VBUS Voltage Comparison', fontsize=14, fontweight='bold')

    colors = ['blue', 'red', 'green', 'orange', 'purple']
    plotted_ports = []

    for port_data in report_data.get("per_port", []):
        port = port_data.get("port")
        power_report = port_data.get("power_report")

        # Skip port 0 and ports without ADC samples
        if port == 0 or not power_report:
            continue

        adc_samples = power_report.get("adc_samples")
        if not adc_samples:
            continue

        # Convert to voltage
        voltages_v = [adc_code_to_vbus_mv(code) / 1000.0 for code in adc_samples]
        time_ms = np.linspace(0, STEP_WINDOW_MS, len(adc_samples))

        # Plot with unique color
        color = colors[port % len(colors)]
        ax.plot(time_ms, voltages_v, linewidth=0.7, color=color,
               alpha=0.7, label=f'Port {port}')
        plotted_ports.append(port)

    if not plotted_ports:
        print("No ports with ADC data to plot")
        plt.close()
        return

    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('VBUS Voltage (V)', fontsize=11)
    ax.set_title(f'ADC Sample Traces for Ports: {", ".join(map(str, plotted_ports))}', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    if output_dir:
        output_path = Path(output_dir) / "all_ports_overlay.png"
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved overlay plot: {output_path}")
        plt.close()
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot ADC samples from USB power test report"
    )
    parser.add_argument("input", nargs="?", default="./usb_report.json",
                       help="Input JSON report file (default: ./usb_report.json)")
    parser.add_argument("-o", "--output-dir", default=None,
                       help="Output directory for plots (default: show plots interactively)")
    parser.add_argument("--port", type=int, default=None,
                       help="Plot only specific port number")
    parser.add_argument("--overlay", action="store_true",
                       help="Create overlay plot of all ports")
    parser.add_argument("--no-individual", action="store_true",
                       help="Skip individual port plots")

    args = parser.parse_args()

    # Load JSON report
    try:
        with open(args.input, 'r') as f:
            report = json.load(f)
    except FileNotFoundError:
        print(f"Error: File '{args.input}' not found")
        return 1
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in '{args.input}': {e}")
        return 1

    # Create output directory if specified
    if args.output_dir:
        output_path = Path(args.output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        print(f"Saving plots to: {output_path}")

    # Plot individual ports
    if not args.no_individual:
        for port_data in report.get("per_port", []):
            port = port_data.get("port")

            # Skip if specific port requested and this isn't it
            if args.port is not None and port != args.port:
                continue

            # Skip port 0 (no power measurement)
            if port == 0:
                continue

            plot_port_adc_samples(port_data, args.output_dir)

    # Create overlay plot if requested
    if args.overlay and args.port is None:
        plot_all_ports_overlay(report, args.output_dir)

    print("Done!")
    return 0


if __name__ == "__main__":
    exit(main())
