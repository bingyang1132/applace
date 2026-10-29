//! Visualization utilities for placement results
//!
//! Provides functions to visualize loss curves and cell positions

use crate::data::PlaceData;
use crate::error::PlaceResult;
use ndarray::Array1;
use std::fs::File;
use std::io::Write;
use std::path::Path;

/// Export loss history to CSV file
///
/// # Arguments
///
/// * `loss_history` - Vector of loss values over iterations
/// * `path` - Output CSV file path
pub fn export_loss_csv<P: AsRef<Path>>(loss_history: &[f64], path: P) -> PlaceResult<()> {
    let mut file = File::create(path)?;
    writeln!(file, "iteration,loss")?;

    for (i, loss) in loss_history.iter().enumerate() {
        writeln!(file, "{},{:.6e}", i, loss)?;
    }

    log::info!(
        "Loss history exported to CSV ({} iterations)",
        loss_history.len()
    );
    Ok(())
}

/// Export cell positions to CSV file
///
/// Position format: [x0, x1, ..., x_{N-1}, y0, y1, ..., y_{N-1}]
///
/// # Arguments
///
/// * `positions` - Position array
/// * `data` - PlaceData for cell information
/// * `path` - Output CSV file path
pub fn export_positions_csv<P: AsRef<Path>>(
    positions: &Array1<f64>,
    data: &PlaceData,
    path: P,
) -> PlaceResult<()> {
    let mut file = File::create(path)?;
    writeln!(file, "cell_id,x,y,width,height")?;

    let num_cells = data.num_cells;
    for i in 0..num_cells {
        let x = positions[i];
        let y = positions[i + num_cells];
        let width = data.cell_widths[i];
        let height = data.cell_heights[i];

        writeln!(file, "{},{:.4},{:.4},{:.4},{:.4}", i, x, y, width, height)?;
    }

    log::info!("Cell positions exported to CSV ({} cells)", num_cells);
    Ok(())
}

/// Generate Python script to plot loss curve
///
/// # Arguments
///
/// * `csv_path` - Path to the loss CSV file
/// * `output_path` - Path to save the Python script
pub fn generate_plot_loss_script<P: AsRef<Path>>(_csv_path: P, output_path: P) -> PlaceResult<()> {
    let script = r#"#!/usr/bin/env python3
"""
Plot loss curve from placement optimization.
CSV and PNG paths default to the same directory as this script.
"""

import matplotlib.pyplot as plt
import pandas as pd
import sys
import os

def plot_loss(csv_file, output_file=None):
    # Read CSV
    df = pd.read_csv(csv_file)
    
    # Create figure
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot loss curve
    ax.plot(df['iteration'], df['loss'], linewidth=2, color='#2E86AB')
    ax.set_xlabel('Iteration', fontsize=12)
    ax.set_ylabel('Loss', fontsize=12)
    ax.set_title('Global Placement Loss Curve', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_yscale('log')
    
    # Add statistics
    min_loss = df['loss'].min()
    final_loss = df['loss'].iloc[-1]
    textstr = 'Final Loss: {{:.4e}}\nMin Loss: {{:.4e}}'.format(final_loss, min_loss)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Loss curve saved to {output_file}")
    else:
        plt.show()

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, 'loss_history.csv')
    output_file = os.path.join(script_dir, 'loss_curve.png')
    
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    
    plot_loss(csv_file, output_file)
"#;

    let mut file = File::create(output_path.as_ref())?;
    file.write_all(script.as_bytes())?;

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut perms = file.metadata()?.permissions();
        perms.set_mode(0o755);
        std::fs::set_permissions(output_path.as_ref(), perms)?;
    }

    log::info!("Plot script generated: {}", output_path.as_ref().display());
    Ok(())
}

/// Generate Python script to visualize cell positions
///
/// # Arguments
///
/// * `csv_path` - Path to the positions CSV file
/// * `output_path` - Path to save the Python script
pub fn generate_plot_positions_script<P: AsRef<Path>>(
    _csv_path: P,
    output_path: P,
) -> PlaceResult<()> {
    let script = r#"#!/usr/bin/env python3
"""
Visualize cell positions from placement.
CSV and PNG paths default to the same directory as this script.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
import sys
import os

def plot_positions(csv_file, output_file=None):
    # Read CSV
    df = pd.read_csv(csv_file)
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Plot each cell as a rectangle
    for _, row in df.iterrows():
        rect = patches.Rectangle(
            (row['x'], row['y']),
            row['width'],
            row['height'],
            linewidth=1,
            edgecolor='#2E86AB',
            facecolor='#A3CEF1',
            alpha=0.6
        )
        ax.add_patch(rect)
        
        # Add cell ID at center
        cx = row['x'] + row['width'] / 2
        cy = row['y'] + row['height'] / 2
        ax.text(cx, cy, str(row['cell_id']), 
                ha='center', va='center', fontsize=6)
    
    # Set axis properties
    ax.set_xlabel('X (um)', fontsize=12)
    ax.set_ylabel('Y (um)', fontsize=12)
    ax.set_title(f'Cell Positions ({len(df)} cells)', 
                 fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # Auto-adjust limits with margin
    margin = 0.1
    x_min, x_max = df['x'].min(), (df['x'] + df['width']).max()
    y_min, y_max = df['y'].min(), (df['y'] + df['height']).max()
    x_range = x_max - x_min
    y_range = y_max - y_min
    ax.set_xlim(x_min - margin * x_range, x_max + margin * x_range)
    ax.set_ylim(y_min - margin * y_range, y_max + margin * y_range)
    
    plt.tight_layout()
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Position visualization saved to {output_file}")
    else:
        plt.show()

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, 'cell_positions.csv')
    output_file = os.path.join(script_dir, 'cell_positions.png')
    
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    
    plot_positions(csv_file, output_file)
"#;

    let mut file = File::create(output_path.as_ref())?;
    file.write_all(script.as_bytes())?;

    // Make executable on Unix
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut perms = file.metadata()?.permissions();
        perms.set_mode(0o755);
        std::fs::set_permissions(output_path.as_ref(), perms)?;
    }

    log::info!("Plot script generated: {}", output_path.as_ref().display());
    Ok(())
}

/// Export all visualization data and scripts
///
/// This is a convenience function that exports:
/// - Loss history CSV
/// - Cell positions CSV
/// - Python plotting scripts
///
/// # Arguments
///
/// * `positions` - Final cell positions
/// * `loss_history` - Loss values over iterations
/// * `data` - PlaceData
/// * `output_dir` - Directory to save outputs
pub fn export_all<P: AsRef<Path>>(
    positions: &Array1<f64>,
    loss_history: &[f64],
    data: &PlaceData,
    output_dir: P,
) -> PlaceResult<()> {
    let dir = output_dir.as_ref();

    // Create output directory if not exists
    std::fs::create_dir_all(dir)?;

    // Export CSVs
    let loss_csv = dir.join("loss_history.csv");
    let pos_csv = dir.join("cell_positions.csv");
    export_loss_csv(loss_history, &loss_csv)?;
    export_positions_csv(positions, data, &pos_csv)?;

    // Generate plotting scripts
    let plot_loss_py = dir.join("plot_loss.py");
    let plot_pos_py = dir.join("plot_positions.py");
    generate_plot_loss_script(&loss_csv, &plot_loss_py)?;
    generate_plot_positions_script(&pos_csv, &plot_pos_py)?;

    log::info!("All visualization data exported to: {}", dir.display());
    log::info!("  - loss_history.csv");
    log::info!("  - cell_positions.csv");
    log::info!("  - plot_loss.py (run to generate loss curve)");
    log::info!("  - plot_positions.py (run to visualize positions)");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_export_loss_csv() {
        let temp_dir = TempDir::new().unwrap();
        let csv_path = temp_dir.path().join("loss.csv");

        let loss_history = vec![1000.0, 500.0, 250.0, 125.0];
        export_loss_csv(&loss_history, &csv_path).unwrap();

        assert!(csv_path.exists());
        let content = std::fs::read_to_string(&csv_path).unwrap();
        assert!(content.contains("iteration,loss"));
        assert!(content.contains("0,1.000000e3"));
    }

    #[test]
    fn test_export_positions_csv() {
        let temp_dir = TempDir::new().unwrap();
        let csv_path = temp_dir.path().join("positions.csv");
        let num_cells = 2usize;

        // Create minimal PlaceData
        let data = PlaceData {
            num_cells,
            num_pins: 0,
            num_nets: 0,
            cell_widths: Array1::from_vec(vec![10.0, 20.0]),
            cell_heights: Array1::from_vec(vec![15.0, 25.0]),
            cell_areas: Array1::from_vec(vec![150.0, 500.0]),
            cell_widths_keepout: Array1::from_vec(vec![10.0, 20.0]),
            cell_heights_keepout: Array1::from_vec(vec![15.0, 25.0]),
            cell_areas_keepout: Array1::from_vec(vec![150.0, 500.0]),
            is_pmos_mask: Array1::from_vec(vec![false, false]),
            pin_offset: Array1::zeros(0),
            pin_owner_cell: Array1::zeros(0),
            io_pin_mask: Array1::from_vec(vec![]),
            net_weights: Array1::zeros(0),
            net_pin_offsets: Array1::zeros(1),
            net_pin_index: Array1::zeros(0),
            cell_pin_offsets: Array1::zeros(3),
            cell_pin_index: Array1::zeros(0),
            overlap_pairs: Array1::zeros(0),
            num_overlap_pairs: 0,
            sym_pairs: Array1::zeros(0),
            sym_pair_group: Array1::zeros(0),
            sym_group_vertical: Array1::from_vec(vec![]),
            sym_group_names: Vec::new(),
            num_sym_pairs: 0,
            num_sym_groups: 0,
            sym_self_weight: 2.0,
            sym_anchor_x: Array1::from_vec(vec![5.0; num_cells]),
            sym_anchor_y: Array1::from_vec(vec![5.0; num_cells]),
            well_group_offsets: Array1::zeros(1),
            well_group_index: Array1::zeros(0),
            cell_well_group: Array1::from_vec(vec![-1, -1]),
            num_clusters: 0,
            collinear_member_index: Array1::zeros(0),
            collinear_member_offsets: Array1::zeros(1),
            collinear_vertical: Array1::from_vec(vec![]),
            num_collinear_groups: 0,
            boundary_xl: 0.0,
            boundary_yl: 0.0,
            boundary_xh: 100.0,
            boundary_yh: 100.0,
        };

        let positions = Array1::from_vec(vec![5.0, 15.0, 10.0, 20.0]); // x0, x1, y0, y1
        export_positions_csv(&positions, &data, &csv_path).unwrap();

        assert!(csv_path.exists());
        let content = std::fs::read_to_string(&csv_path).unwrap();
        assert!(content.contains("cell_id,x,y,width,height"));
        assert!(content.contains("0,5.0000,10.0000,10.0000,15.0000"));
    }
}
