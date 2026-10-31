"""ASCII Terminal Plotter and HTML Chart Generator for Step Response.

Zero-external-dependency visualization:
  - ascii_plot: Renders clean ASCII curves in standard terminal.
  - export_html_chart: Exports self-contained interactive HTML/SVG graph.
"""

from typing import List


def ascii_plot(
    time_series: List[float],
    values: List[float],
    setpoint: float,
    width: int = 60,
    height: int = 15,
) -> str:
    """Render an ASCII graph showing process variable and setpoint line."""
    if not values:
        return ""

    min_v = min(0.0, min(values))
    max_v = max(setpoint * 1.15, max(values))
    span = max_v - min_v
    if span <= 1e-6:
        span = 1.0

    # Downsample time series to fit 'width' columns
    step = len(values) / float(width)
    sampled = []
    for col in range(width):
        idx = min(len(values) - 1, int(col * step))
        sampled.append(values[idx])

    grid = [[" " for _ in range(width)] for _ in range(height)]

    # Draw setpoint dashed line
    sp_row = int((max_v - setpoint) / span * (height - 1))
    if 0 <= sp_row < height:
        for c in range(width):
            grid[sp_row][c] = "-"

    # Draw response points
    for c, val in enumerate(sampled):
        row = int((max_v - val) / span * (height - 1))
        row = max(0, min(height - 1, row))
        grid[row][c] = "*" if grid[row][c] == " " else "+"

    lines = []
    lines.append(f"   Max: {max_v:6.1f} |" + "".join(grid[0]))
    for r in range(1, height - 1):
        label = f"   SP:  {setpoint:6.1f} |" if r == sp_row else " " * 15 + "|"
        lines.append(label + "".join(grid[r]))
    lines.append(f"   Min: {min_v:6.1f} |" + "".join(grid[-1]))
    lines.append(" " * 16 + "+" + "-" * width)
    lines.append(f"{' ' * 16}0.0 s{' ' * (width - 12)}{time_series[-1]:.2f} s")
    lines.append("   Legend: [*] Process Variable (PV)   [-] Target Setpoint (SP)")

    return "\n".join(lines)


def export_html_chart(
    time_series: List[float],
    values: List[float],
    control_efforts: List[float],
    setpoint: float,
    title: str,
    output_path: str,
) -> None:
    """Export an interactive HTML report with embedded responsive Canvas chart."""
    t_json = [round(t, 3) for t in time_series]
    v_json = [round(v, 2) for v in values]
    u_json = [round(u, 2) for u in control_efforts]

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>{title}</title>
  <style>
    body {{
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: #0d1117;
      color: #c9d1d9;
      padding: 24px;
      margin: 0;
    }}
    .card {{
      background: #161b22;
      border: 1px solid #30363d;
      border-radius: 8px;
      padding: 20px;
      max-width: 900px;
      margin: 0 auto;
    }}
    h1 {{ color: #58a6ff; font-size: 1.4rem; margin-top: 0; }}
    canvas {{ width: 100%; height: 360px; background: #0b0e14; border-radius: 6px; }}
    .badge {{ display: inline-block; padding: 4px 10px; border-radius: 12px; font-size: 0.8rem; background: #238636; color: #fff; }}
  </style>
</head>
<body>
  <div class="card">
    <span class="badge">PID Closed-Loop Step Response</span>
    <h1>{title}</h1>
    <canvas id="chart"></canvas>
  </div>
  <script>
    const times = {t_json};
    const pvs = {v_json};
    const sp = {setpoint};
    const cvs = document.getElementById('chart');
    const ctx = cvs.getContext('2d');
    cvs.width = cvs.clientWidth * window.devicePixelRatio;
    cvs.height = cvs.clientHeight * window.devicePixelRatio;
    ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

    const w = cvs.clientWidth, h = cvs.clientHeight;
    const pad = 40;
    const maxVal = Math.max(sp * 1.25, ...pvs);
    const minVal = 0;

    const toX = (t) => pad + (t / times[times.length - 1]) * (w - 2 * pad);
    const toY = (v) => h - pad - ((v - minVal) / (maxVal - minVal)) * (h - 2 * pad);

    // Draw Setpoint line
    ctx.strokeStyle = '#f85149';
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(pad, toY(sp));
    ctx.lineTo(w - pad, toY(sp));
    ctx.stroke();

    // Draw Process Variable
    ctx.strokeStyle = '#2f81f7';
    ctx.lineWidth = 2.5;
    ctx.setLineDash([]);
    ctx.beginPath();
    times.forEach((t, i) => {{
      const x = toX(t), y = toY(pvs[i]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }});
    ctx.stroke();

    // Labels
    ctx.fillStyle = '#8b949e';
    ctx.font = '12px sans-serif';
    ctx.fillText('0 s', pad, h - 15);
    ctx.fillText(times[times.length - 1] + ' s', w - pad - 30, h - 15);
    ctx.fillText('SP = ' + sp, pad + 10, toY(sp) - 8);
  </script>
</body>
</html>
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html_content)
