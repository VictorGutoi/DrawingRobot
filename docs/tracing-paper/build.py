#!/usr/bin/env python3
"""Convierte un archivo .md en .pdf listo para imprimir.

Uso:
    python3 build.py                 # por defecto: teoria.md
    python3 build.py apuntes.md      # cualquier .md en este directorio

Pipeline:
  1. Lee el Markdown (ignora el frontmatter YAML).
  2. Lo convierte a HTML con python-markdown + extensiones (toc, tables, extra).
  3. Envuelve el HTML con MathJax (para $...$ y $$...$$) y un CSS imprimible.
  4. Llama a Google Chrome en modo headless para imprimir a PDF.

No requiere pandoc ni LaTeX.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

import markdown  # ya disponible en el Python del sistema

HERE = Path(__file__).resolve().parent
DEFAULT_MD = "teoria.md"

CHROME = Path("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome")


def strip_yaml_frontmatter(text: str) -> tuple[dict[str, str], str]:
    """Quita un bloque YAML `---...---` al principio y devuelve (meta, body)."""
    if not text.startswith("---"):
        return {}, text
    match = re.match(r"^---\n(.*?)\n---\n", text, re.DOTALL)
    if not match:
        return {}, text
    frontmatter = match.group(1)
    body = text[match.end():]
    meta: dict[str, str] = {}
    for line in frontmatter.splitlines():
        if ":" in line:
            k, _, v = line.partition(":")
            meta[k.strip()] = v.strip().strip('"')
    return meta, body


def render_html(meta: dict[str, str], body_html: str) -> str:
    title = meta.get("title", "Material de estudio")
    subtitle = meta.get("subtitle", "")
    author = meta.get("author", "")
    return f"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<title>{title}</title>
<script>
MathJax = {{
  tex: {{
    inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
    displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
    processEscapes: true
  }},
  svg: {{ fontCache: 'global' }}
}};
</script>
<script id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
<style>
@page {{
  size: A4;
  margin: 2cm 2cm 2.2cm 2cm;
}}
@page :first {{
  margin: 0;
}}
body {{
  font-family: "Georgia", "Times New Roman", serif;
  font-size: 10.8pt;
  line-height: 1.45;
  color: #1f2328;
  max-width: none;
  margin: 0;
  padding: 0;
  hyphens: auto;
  text-align: justify;
}}
.cover {{
  page: cover;
  text-align: center;
  padding: 10cm 2cm 0 2cm;
  page-break-after: always;
  min-height: 29cm;
}}
.cover h1.title {{
  font-size: 30pt;
  color: #0b2a57;
  border: none;
  margin: 0 0 0.4em 0;
  padding: 0;
  page-break-before: avoid;
  text-align: center;
}}
.cover .subtitle {{
  font-size: 16pt;
  color: #365078;
  font-style: italic;
  margin-bottom: 2em;
}}
.cover .author {{
  font-size: 12pt;
  color: #555;
  margin-top: 4em;
}}
h1, h2, h3, h4 {{
  font-family: "Helvetica Neue", "Arial", sans-serif;
  color: #0b2a57;
  page-break-after: avoid;
  line-height: 1.25;
}}
h1 {{
  font-size: 22pt;
  border-bottom: 3px solid #0b2a57;
  padding-bottom: 0.25em;
  margin: 1.2em 0 0.8em 0;
  page-break-before: always;
}}
h1:first-child {{ page-break-before: avoid; }}
h2 {{
  font-size: 15pt;
  margin: 1.4em 0 0.5em 0;
  border-left: 4px solid #0b2a57;
  padding-left: 0.4em;
}}
h3 {{
  font-size: 12.5pt;
  margin: 1.1em 0 0.3em 0;
  color: #1a3a6b;
}}
h4 {{ font-size: 11pt; color: #1a3a6b; }}
p {{ margin: 0.5em 0; }}
strong {{ color: #0b2a57; }}
em {{ color: #444; }}
code {{
  font-family: "Menlo", "Courier New", monospace;
  font-size: 9.5pt;
  background: #f3f5f8;
  padding: 0.1em 0.3em;
  border-radius: 3px;
}}
blockquote {{
  border-left: 4px solid #ffb703;
  margin: 1em 0;
  padding: 0.2em 0.8em;
  background: #fff8e3;
  color: #3b2f00;
  font-size: 10.4pt;
  page-break-inside: avoid;
}}
table {{
  border-collapse: collapse;
  margin: 0.8em 0;
  font-size: 10pt;
  page-break-inside: avoid;
  width: 100%;
}}
th, td {{
  border: 1px solid #999;
  padding: 0.35em 0.6em;
  text-align: left;
}}
th {{
  background: #e8eef5;
  color: #0b2a57;
  font-weight: bold;
}}
ul, ol {{ margin: 0.4em 0 0.6em 1.2em; padding: 0; }}
li {{ margin: 0.15em 0; }}
hr {{
  border: none;
  border-top: 1px solid #aaa;
  margin: 1em 0;
}}
.toc {{
  page-break-after: always;
  padding: 2em 1em;
}}
.toc h2 {{
  font-size: 22pt;
  border: none;
  border-left: 4px solid #0b2a57;
  margin-top: 0;
}}
.toc ul {{
  list-style: none;
  padding-left: 0.6em;
}}
.toc a {{ color: #0b2a57; text-decoration: none; }}
a {{ color: #0b2a57; }}
mjx-container {{ font-size: 100% !important; }}
mjx-container[jax="CHTML"][display="true"] {{
  margin: 0.4em 0 !important;
  text-align: center !important;
}}
</style>
</head>
<body>
<div class="cover">
  <h1 class="title">{title}</h1>
  <div class="subtitle">{subtitle}</div>
  <div class="author">{author}</div>
</div>
{body_html}
</body>
</html>
"""


def build_html(md_path: Path, html_path: Path) -> None:
    if not md_path.exists():
        sys.exit(f"No se encuentra {md_path}")

    raw = md_path.read_text(encoding="utf-8")
    meta, body_md = strip_yaml_frontmatter(raw)

    # Markdown de Python no entiende `\newpage` de LaTeX — lo sustituimos por
    # un div con page-break-before para que Chrome haga el salto de página.
    body_md = body_md.replace("\\newpage", '<div style="page-break-before: always"></div>')

    md = markdown.Markdown(extensions=[
        "extra",       # tablas, listas anidadas, fenced_code, etc.
        "toc",         # genera [TOC] y anclas en cabeceras
        "sane_lists",  # listas mas predecibles
    ], extension_configs={
        "toc": {"title": "Índice", "toc_depth": "1-3"},
    })

    # Insertamos el TOC al principio automaticamente.
    body_md = "[TOC]\n\n" + body_md
    body_html = md.convert(body_md)

    html = render_html(meta, body_html)
    html_path.write_text(html, encoding="utf-8")
    print(f"✓ HTML generado: {html_path.relative_to(HERE)}")


def build_pdf(html_path: Path, pdf_path: Path) -> None:
    if not CHROME.is_file():
        sys.exit(f"No se encuentra Google Chrome en {CHROME}")

    # Chrome headless. virtual-time-budget da a MathJax tiempo para renderizar.
    cmd = [
        str(CHROME),
        "--headless=new",
        "--disable-gpu",
        "--no-sandbox",
        "--hide-scrollbars",
        "--virtual-time-budget=20000",
        "--run-all-compositor-stages-before-draw",
        f"--print-to-pdf={pdf_path}",
        "--no-pdf-header-footer",
        f"file://{html_path}",
    ]
    print("▶ Renderizando PDF con Chrome headless (esto tarda unos segundos)…")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Chrome stderr:", result.stderr, file=sys.stderr)
        sys.exit(result.returncode)

    if not pdf_path.exists():
        sys.exit("Chrome terminó pero no se creó el PDF.")

    size_mb = pdf_path.stat().st_size / 1024 / 1024
    print(f"✓ PDF generado: {pdf_path.relative_to(HERE)} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    md_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MD
    md_path = HERE / md_name
    html_path = md_path.with_suffix(".html")
    pdf_path = md_path.with_suffix(".pdf")

    build_html(md_path, html_path)
    build_pdf(html_path, pdf_path)
