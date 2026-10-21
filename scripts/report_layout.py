from pathlib import Path
import csv
import io
import textwrap
import json
import re
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
from matplotlib.transforms import Bbox

NAVY, TEAL = "#153047", "#087e8b"
BLACK = "#000000"
TITLE_SIZE, HEADING_SIZE = 17, 16


def heading_level(title, default=1):
    match = re.match(r"^(?:Appendix\s+)?(\d+(?:\.\d+)*|[A-Z](?:\.\d+)*)(?:\.\s|\s|$)", title)
    return match.group(1).count(".")+1 if match else default


class Report:
    def __init__(self, output):
        self.output = Path(output)
        self.pdf = PdfPages(self.output/"report.pdf", metadata={
            "Title": "Sonar-guided underwater survey with BlueROV2",
            "Author": "Yecheng Wang",
            "Subject": "LTU-RAI PhD Recruitment Assignment; Date: 21 October 2026"})
        title = "Sonar-guided underwater survey with BlueROV2"
        self.rst = [title, "="*len(title), "",
                    "Yecheng Wang | LTU-RAI PhD Recruitment Assignment | 21 October 2026", ""]
        self.fig = None
        self.blocks = []
        self.page_number = self.table_number = self.figure_number = self.equation_number = 0

    def page(self, title):
        self.flush()
        self.page_number += 1
        self.fig = plt.figure(figsize=(8.27, 11.69), facecolor="white")
        self.section_level = 0 if self.page_number == 1 else heading_level(title)
        size = TITLE_SIZE if self.page_number == 1 else HEADING_SIZE-self.section_level+1
        self.fig.text(.08, .913, title, fontsize=size, weight="bold", color=NAVY, va="top")
        self.fig.text(.08, .042, "Yecheng Wang  |  BlueROV2 survey  |  21 October 2026",
                      fontsize=7.5, color="#647582")
        self.fig.text(.92, .042, str(self.page_number), fontsize=8, color="#647582", ha="right")
        self.y = .867
        self.rst += [title, "-"*len(title), ""]
        self.blocks.append(dict(kind="page", title=title))

    def flush(self):
        if self.fig is not None:
            self.pdf.savefig(self.fig)
            plt.close(self.fig)
            self.fig = None

    def ensure(self, height):
        if self.y-height < .082:
            raise RuntimeError(f"Report page {self.page_number} overflows at y={self.y:.3f}, height={height:.3f}")

    def lines(self, text, fontsize=10, width=.84, family="sans-serif"):
        renderer = self.fig.canvas.get_renderer()
        prop = FontProperties(size=fontsize, family=family)
        max_width = width*self.fig.get_figwidth()*self.fig.dpi
        lines = []
        for paragraph in text.splitlines():
            current = ""
            for word in paragraph.split():
                trial = (current+" "+word).strip()
                if renderer.get_text_width_height_descent(trial, prop, False)[0] > max_width and current:
                    lines.append(current)
                    current = word
                else:
                    current = trial
            lines.append(current)
        return lines

    def p(self, text, fontsize=10, align="left"):
        lines = self.lines(text, fontsize)
        step = .0178 if fontsize >= 10 else .016
        self.ensure(len(lines)*step+.014)
        for line in lines:
            self.fig.text(.5 if align == "center" else .08, self.y, line,
                          fontsize=fontsize, color=BLACK, va="top", ha=align)
            self.y -= step
        self.y -= .014
        self.rst.extend([text, ""])
        self.blocks.append(dict(kind="paragraph", text=text, fontsize=fontsize))

    def sub(self, title):
        self.ensure(.044)
        level = heading_level(title, default=max(1, self.section_level+1))
        self.fig.text(.08, self.y, title, fontsize=HEADING_SIZE-level+1,
                      color=NAVY, weight="bold", va="top")
        self.y -= .032
        self.rst.extend([title, "~"*len(title), ""])
        self.blocks.append(dict(kind="subheading", text=title))

    def eq(self, math, height=.045, size=12):
        self.equation_number += 1
        self.ensure(height)
        self.fig.text(.095, self.y-.012, "$"+math+"$", fontsize=size, color=BLACK, va="top")
        self.fig.text(.92, self.y-.015, f"({self.equation_number})", fontsize=10, ha="right", va="top", color=BLACK)
        self.y -= height
        self.rst.extend([".. math::", "", "    "+math, ""])
        self.blocks.append(dict(kind="equation", math=math, number=self.equation_number, height=height))

    def table(self, title, columns, rows, widths=None, fontsize=8.3):
        self.table_number += 1
        widths = widths or [1/len(columns)]*len(columns)
        wrapped, row_heights = [], []
        for row in [columns]+rows:
            line_counts, cells = [], []
            for text, width in zip(row, widths):
                lines = self.lines(str(text), fontsize, width=.84*width*.88)
                cells.append("\n".join(lines))
                line_counts.append(len(lines))
            wrapped.append(cells)
            row_heights.append(max(line_counts)*(.0125 if fontsize < 8 else .0145)+.012)
        height = sum(row_heights)
        self.ensure(height+.049)
        caption = f"Table {self.table_number}. {title}"
        ax = self.fig.add_axes([.08, self.y-height, .84, height])
        ax.axis("off")
        table = ax.table(cellText=wrapped, colWidths=widths, cellLoc="left", loc="center", bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(fontsize)
        for (row, col), cell in table.get_celld().items():
            cell.set_height(row_heights[row]/height)
            cell.set_linewidth(.45)
            cell.set_edgecolor("#c5d2da")
            cell.PAD = .06
            cell.set_facecolor(TEAL if row == 0 else ("#f0f5f7" if row % 2 else "white"))
            cell.set_text_props(color="white" if row == 0 else BLACK, weight="bold" if row == 0 else "normal")
        self.y -= height+.012
        self.fig.text(.5, self.y, caption, color=BLACK, fontsize=9,
                      weight="bold", va="top", ha="center")
        self.y -= .037
        text = io.StringIO()
        writer = csv.writer(text)
        writer.writerow(columns)
        writer.writerows(rows)
        self.rst.extend([f".. csv-table:: {caption}", "   :header-rows: 1", "",
                         textwrap.indent(text.getvalue().rstrip(), "   "), ""])
        self.blocks.append(dict(kind="table", caption=caption, columns=columns, rows=rows,
                                widths=widths, fontsize=fontsize))

    def code(self, text, fontsize=8):
        lines = text.splitlines()
        height = len(lines)*.015+.025
        self.ensure(height)
        self.fig.patches.append(plt.Rectangle((.075, self.y-height), .85, height,
                                facecolor="#f0f4f6", edgecolor="#d9e3e8",
                                transform=self.fig.transFigure, zorder=-1))
        for line in lines:
            self.fig.text(.09, self.y-.01, line, family="monospace", fontsize=fontsize, color=BLACK, va="top")
            self.y -= .015
        self.y -= .04
        self.rst.extend(["::", "", textwrap.indent(text, "    "), ""])
        self.blocks.append(dict(kind="code", text=text, fontsize=fontsize))

    def figure(self, name, draw, run, height, caption):
        self.figure_number += 1
        self.ensure(height+.095)
        before = set(self.fig.axes)
        draw(self.fig, (.13, self.y-height, .75, height), run)
        self.fig.canvas.draw()
        renderer = self.fig.canvas.get_renderer()
        boxes = [ax.get_tightbbox(renderer) for ax in self.fig.axes if ax not in before]
        crop = Bbox.union(boxes).transformed(self.fig.dpi_scale_trans.inverted()).padded(.035)
        figure_path = self.output / "figures" / (name+"_report.png")
        self.fig.savefig(figure_path, bbox_inches=crop, dpi=300)
        self.blocks.append(dict(kind="figure", name=name, path=str(figure_path),
                                width_inches=crop.width, height_inches=crop.height))
        self.y -= height+(.085 if name == "coverage" else .045)
        self.p(f"Figure {self.figure_number}. {caption}", fontsize=9, align="center")
        self.rst.extend([f".. image:: figures/{name}.pdf", "   :width: 100%", ""])

    def link(self, title, url):
        self.ensure(.038)
        self.fig.text(.08, self.y, title, fontsize=9.5, color=BLACK, va="top", url=url)
        self.y -= .027
        mark = chr(96)
        self.rst.extend([f"{mark}{title} <{url}>{mark}__", ""])
        self.blocks.append(dict(kind="link", text=title, url=url))

    def finish(self):
        self.flush()
        self.pdf.close()
        (self.output/"report.rst").write_text("\n".join(self.rst))
        (self.output/"report_content.json").write_text(json.dumps(self.blocks, indent=2)+"\n")
