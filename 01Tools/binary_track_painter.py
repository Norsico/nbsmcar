#!/usr/bin/env python3
"""
二值赛道画板。

默认画布为 20x15：
- 0 表示白色赛道
- 1 表示黑色背景
- * 表示红色中线标记
- | 表示绿色边界标记

运行方式（Windows）：
    py -3 01Tools\\binary_track_painter.py

也可自定义尺寸：
    py -3 01Tools\\binary_track_painter.py --cols 80 --rows 60 --cell-size 10
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import tempfile
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, messagebox


CELL_WHITE = 0
CELL_BLACK = 1
CELL_RED = 2
CELL_GREEN = 3

COLOR_WHITE = "#ffffff"
COLOR_BLACK = "#000000"
COLOR_RED = "#ff3b30"
COLOR_GREEN = "#34c759"
GRID_LINE = "#303030"

CELL_TO_COLOR = {
    CELL_WHITE: COLOR_WHITE,
    CELL_BLACK: COLOR_BLACK,
    CELL_RED: COLOR_RED,
    CELL_GREEN: COLOR_GREEN,
}

CELL_TO_TEXT = {
    CELL_WHITE: "0",
    CELL_BLACK: "1",
    CELL_RED: "*",
    CELL_GREEN: "|",
}


class BinaryTrackPainter:
    def __init__(self, root: tk.Tk, rows: int, cols: int, cell_size: int) -> None:
        self.root = root
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.rows_var = tk.IntVar(value=rows)
        self.cols_var = tk.IntVar(value=cols)
        self.grid = [[CELL_BLACK for _ in range(cols)] for _ in range(rows)]
        self.rectangles: list[list[int]] = [[0 for _ in range(cols)] for _ in range(rows)]
        self.brush_size = tk.IntVar(value=1)
        self.current_brush = tk.IntVar(value=CELL_WHITE)
        self.canvas_size_text = tk.StringVar()
        self.status_text = tk.StringVar(value="左键按当前画笔绘制，右键快速擦回黑色(1)。")
        self.display_cell_size = cell_size
        self._resize_after_id = None

        self._build_ui()
        self._rebuild_canvas()

    def _build_ui(self) -> None:
        self.root.title("Binary Track Painter")
        self.root.geometry("860x620")
        self.root.minsize(520, 420)

        toolbar = tk.Frame(self.root)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=8, pady=8)

        tk.Label(toolbar, text="画笔").pack(side=tk.LEFT)
        tk.Spinbox(
            toolbar,
            from_=1,
            to=8,
            width=4,
            textvariable=self.brush_size,
        ).pack(side=tk.LEFT, padx=(4, 12))

        tk.Radiobutton(toolbar, text="白(0)", variable=self.current_brush, value=CELL_WHITE).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(toolbar, text="黑(1)", variable=self.current_brush, value=CELL_BLACK).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(toolbar, text="红(*)", variable=self.current_brush, value=CELL_RED).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(toolbar, text="绿(|)", variable=self.current_brush, value=CELL_GREEN).pack(side=tk.LEFT, padx=2)

        tk.Label(toolbar, text="列").pack(side=tk.LEFT, padx=(12, 2))
        tk.Spinbox(toolbar, from_=1, to=200, width=5, textvariable=self.cols_var).pack(side=tk.LEFT, padx=(0, 6))
        tk.Label(toolbar, text="行").pack(side=tk.LEFT, padx=(0, 2))
        tk.Spinbox(toolbar, from_=1, to=200, width=5, textvariable=self.rows_var).pack(side=tk.LEFT, padx=(0, 6))
        tk.Button(toolbar, text="重建画布", command=self.apply_canvas_resolution).pack(side=tk.LEFT, padx=(0, 8))

        tk.Button(toolbar, text="全黑重置", command=self.fill_black).pack(side=tk.LEFT, padx=4)
        tk.Button(toolbar, text="全白填充", command=self.fill_white).pack(side=tk.LEFT, padx=4)
        tk.Button(toolbar, text="复制文本", command=self.copy_matrix_text).pack(side=tk.LEFT, padx=4)
        tk.Button(toolbar, text="复制图片", command=self.copy_image_to_clipboard).pack(side=tk.LEFT, padx=4)
        tk.Button(toolbar, text="保存 JPG", command=self.save_jpg).pack(side=tk.LEFT, padx=4)

        tk.Label(
            toolbar,
            textvariable=self.canvas_size_text,
        ).pack(side=tk.LEFT, padx=(16, 0))

        self.canvas_frame = tk.Frame(self.root, bg="#202020")
        self.canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.canvas_frame.bind("<Configure>", self._on_canvas_frame_resize)
        self.canvas = None

        status = tk.Label(
            self.root,
            textvariable=self.status_text,
            anchor="w",
            relief=tk.SUNKEN,
            padx=8,
        )
        status.pack(side=tk.BOTTOM, fill=tk.X)

    def _update_canvas_size_text(self) -> None:
        self.canvas_size_text.set(
            f"当前尺寸 {self.cols}x{self.rows}，0=白，1=黑，*=红，|=绿"
        )

    def _rebuild_canvas(self) -> None:
        if self.canvas is not None:
            self.canvas.destroy()

        self.grid = [[CELL_BLACK for _ in range(self.cols)] for _ in range(self.rows)]
        self.rectangles = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        self.canvas = tk.Canvas(
            self.canvas_frame,
            width=1,
            height=1,
            bg=COLOR_BLACK,
            highlightthickness=0,
        )
        self.canvas.pack(side=tk.TOP, fill=tk.NONE, expand=True)
        self.canvas.bind("<Button-1>", self._paint_selected)
        self.canvas.bind("<B1-Motion>", self._paint_selected)
        self.canvas.bind("<Button-3>", self._paint_black)
        self.canvas.bind("<B3-Motion>", self._paint_black)
        self._redraw_canvas()
        self._update_canvas_size_text()

    def apply_canvas_resolution(self) -> None:
        rows = self.rows_var.get()
        cols = self.cols_var.get()

        if rows <= 0 or cols <= 0:
            messagebox.showerror("重建画布失败", "行和列都必须大于 0。")
            return

        self.rows = rows
        self.cols = cols
        self._rebuild_canvas()
        self.status_text.set(f"已按 {self.cols}x{self.rows} 重建画布。")

    def _get_display_cell_size(self) -> int:
        frame_width = max(1, self.canvas_frame.winfo_width() - 8)
        frame_height = max(1, self.canvas_frame.winfo_height() - 8)
        fit_by_width = max(1, frame_width // self.cols)
        fit_by_height = max(1, frame_height // self.rows)
        return max(1, min(self.cell_size, fit_by_width, fit_by_height))

    def _redraw_canvas(self) -> None:
        row = 0
        col = 0
        x0 = 0
        y0 = 0
        x1 = 0
        y1 = 0
        outline = GRID_LINE

        if self.canvas is None:
            return

        self.display_cell_size = self._get_display_cell_size()
        if(self.display_cell_size <= 4):
            outline = ""

        self.canvas.delete("all")
        self.canvas.config(
            width=self.cols * self.display_cell_size,
            height=self.rows * self.display_cell_size,
        )
        self.rectangles = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

        for row in range(self.rows):
            for col in range(self.cols):
                x0 = col * self.display_cell_size
                y0 = row * self.display_cell_size
                x1 = x0 + self.display_cell_size
                y1 = y0 + self.display_cell_size
                self.rectangles[row][col] = self.canvas.create_rectangle(
                    x0,
                    y0,
                    x1,
                    y1,
                    outline=outline,
                    width=1,
                    fill=CELL_TO_COLOR[self.grid[row][col]],
                )

    def _on_canvas_frame_resize(self, event: tk.Event) -> None:
        del event
        if self._resize_after_id is not None:
            self.root.after_cancel(self._resize_after_id)
        self._resize_after_id = self.root.after(30, self._handle_canvas_frame_resize)

    def _handle_canvas_frame_resize(self) -> None:
        self._resize_after_id = None
        if self.canvas is None:
            return
        next_cell_size = self._get_display_cell_size()
        if next_cell_size != self.display_cell_size:
            self._redraw_canvas()

    def _draw_full_grid(self) -> None:
        for row in range(self.rows):
            for col in range(self.cols):
                x0 = col * self.display_cell_size
                y0 = row * self.display_cell_size
                x1 = x0 + self.display_cell_size
                y1 = y0 + self.display_cell_size
                rect_id = self.canvas.create_rectangle(
                    x0,
                    y0,
                    x1,
                    y1,
                    outline=GRID_LINE,
                    width=1,
                    fill=COLOR_BLACK,
                )
                self.rectangles[row][col] = rect_id

    def _cell_from_event(self, event: tk.Event) -> tuple[int, int] | None:
        col = event.x // self.display_cell_size
        row = event.y // self.display_cell_size
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return None
        return row, col

    def _apply_brush(self, center_row: int, center_col: int, value: int) -> None:
        radius = max(0, self.brush_size.get() - 1)
        row_min = max(0, center_row - radius)
        row_max = min(self.rows - 1, center_row + radius)
        col_min = max(0, center_col - radius)
        col_max = min(self.cols - 1, center_col + radius)

        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                self.grid[row][col] = value
                self.canvas.itemconfig(
                    self.rectangles[row][col],
                    fill=CELL_TO_COLOR[value],
                )

    def _paint_selected(self, event: tk.Event) -> None:
        cell = self._cell_from_event(event)
        if cell is None:
            return
        brush = self.current_brush.get()
        self._apply_brush(cell[0], cell[1], brush)
        self.status_text.set(f"正在绘制 {CELL_TO_TEXT[brush]} 标记。")

    def _paint_black(self, event: tk.Event) -> None:
        cell = self._cell_from_event(event)
        if cell is None:
            return
        self._apply_brush(cell[0], cell[1], CELL_BLACK)
        self.status_text.set("正在擦回黑色背景，当前写入 1。")

    def fill_black(self) -> None:
        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[row][col] = CELL_BLACK
                self.canvas.itemconfig(self.rectangles[row][col], fill=COLOR_BLACK)
        self.status_text.set("已重置为全黑背景。")

    def fill_white(self) -> None:
        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[row][col] = CELL_WHITE
                self.canvas.itemconfig(self.rectangles[row][col], fill=COLOR_WHITE)
        self.status_text.set("已填充为全白。")

    def build_matrix_text(self) -> str:
        lines = []
        for row in self.grid:
            lines.append(" ".join(CELL_TO_TEXT[value] for value in row))
        return "\n".join(lines)

    def copy_matrix_text(self) -> None:
        matrix_text = self.build_matrix_text()
        self.root.clipboard_clear()
        self.root.clipboard_append(matrix_text)
        self.root.update()
        self.status_text.set("文本矩阵已复制到剪贴板。")

    def _build_render_image(self, scale: int = 10) -> tk.PhotoImage:
        image = tk.PhotoImage(width=self.cols * scale, height=self.rows * scale)
        image.put(COLOR_BLACK, to=(0, 0, self.cols * scale, self.rows * scale))

        for row in range(self.rows):
            for col in range(self.cols):
                color = CELL_TO_COLOR[self.grid[row][col]]
                x0 = col * scale
                y0 = row * scale
                x1 = x0 + scale
                y1 = y0 + scale
                image.put(color, to=(x0, y0, x1, y1))

        return image

    def _write_temp_image(self) -> pathlib.Path:
        image = self._build_render_image(scale=max(2, self.cell_size))
        temp_dir = pathlib.Path(tempfile.mkdtemp(prefix="binary_track_painter_"))
        png_path = temp_dir / "track.png"
        image.write(str(png_path), format="png")
        return png_path

    def _run_powershell(self, script: str, *args: str) -> subprocess.CompletedProcess[str]:
        command = [
            "powershell.exe",
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-STA",
            "-Command",
            script,
            *args,
        ]
        return subprocess.run(command, capture_output=True, text=True, check=False)

    def copy_image_to_clipboard(self) -> None:
        try:
            png_path = self._write_temp_image()
        except Exception as exc:
            messagebox.showerror("复制图片失败", f"生成临时图片失败：\n{exc}")
            return

        ps_script = r"""
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
$imgPath = $args[0]
$img = [System.Drawing.Image]::FromFile($imgPath)
try {
    [System.Windows.Forms.Clipboard]::SetImage($img)
}
finally {
    $img.Dispose()
}
"""
        result = self._run_powershell(ps_script, str(png_path))
        if result.returncode != 0:
            messagebox.showerror(
                "复制图片失败",
                "Windows 剪贴板写入失败。\n\n"
                f"{result.stderr.strip() or result.stdout.strip()}",
            )
            return

        self.status_text.set("图片已复制到 Windows 剪贴板。")

    def save_jpg(self) -> None:
        default_name = f"binary_track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        save_path = filedialog.asksaveasfilename(
            title="保存 JPG",
            defaultextension=".jpg",
            initialfile=default_name,
            filetypes=[("JPEG", "*.jpg"), ("JPEG", "*.jpeg")],
        )
        if not save_path:
            return

        try:
            png_path = self._write_temp_image()
        except Exception as exc:
            messagebox.showerror("保存 JPG 失败", f"生成临时图片失败：\n{exc}")
            return

        ps_script = r"""
Add-Type -AssemblyName System.Drawing
$src = $args[0]
$dst = $args[1]
$img = [System.Drawing.Image]::FromFile($src)
try {
    $img.Save($dst, [System.Drawing.Imaging.ImageFormat]::Jpeg)
}
finally {
    $img.Dispose()
}
"""
        result = self._run_powershell(ps_script, str(png_path), save_path)
        if result.returncode != 0:
            messagebox.showerror(
                "保存 JPG 失败",
                f"{result.stderr.strip() or result.stdout.strip()}",
            )
            return

        self.status_text.set(f"JPG 已保存到：{save_path}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Binary track painter")
    parser.add_argument("--rows", type=int, default=15, help="行数，默认 15")
    parser.add_argument("--cols", type=int, default=20, help="列数，默认 20")
    parser.add_argument("--cell-size", type=int, default=28, help="单格像素大小，默认 28")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.rows <= 0 or args.cols <= 0 or args.cell_size <= 0:
        print("rows / cols / cell-size 必须大于 0", file=sys.stderr)
        return 1

    root = tk.Tk()
    app = BinaryTrackPainter(root, rows=args.rows, cols=args.cols, cell_size=args.cell_size)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
