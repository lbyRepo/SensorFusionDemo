#!/usr/bin/env python3
"""
Build the single-page GitHub Pages course site (index.html) from docs/*.md.

Dependency-free on purpose: implements the small Markdown subset used by the
lesson files (headers, tables, fenced code, blockquotes, lists, images, links,
details/summary), embeds every figure as base64 so index.html is fully
self-contained, and renders math client-side via MathJax (CDN).

Usage:  python3 tools/build_site.py     (from the repository root)
"""

import base64
import html
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parent.parent

LESSONS = [
    ("01-why-sensor-fusion.md", "lesson-1", "Why Sensor Fusion?"),
    ("02-ekf-foundations.md", "lesson-2", "EKF Foundations"),
    ("03-1d-linear-walkthrough.md", "lesson-3", "1D Linear Walkthrough"),
    ("04-monte-carlo-validation.md", "lesson-4", "Monte Carlo Validation"),
    ("05-2d-nonlinear-walkthrough.md", "lesson-5", "2D Nonlinear Walkthrough"),
    ("06-observability-and-limits.md", "lesson-6", "Observability and Limits"),
    ("07-summary-and-exercises.md", "lesson-7", "Summary and Next Steps"),
]

ANCHORS = {fname: f"#{sid}" for fname, sid, _ in LESSONS}


# --------------------------------------------------------------------------
# EKF predict/correct loop (replaces the mermaid block in lesson 1)
# --------------------------------------------------------------------------

MERMAID_HTML = """
<div class="flow">
  <div class="flow-box flow-sensor">IMU &middot; 100 Hz</div>
  <div class="flow-arrow">&darr; <span>propagate</span></div>
  <div class="flow-box"><strong>PREDICT</strong><span class="flow-eq">x&#770; = F x&#770;<br>P = F P F&#7488; + Q</span></div>
  <div class="flow-arrow">&darr;</div>
  <div class="flow-box flow-decision">Aiding measurement available?</div>
  <div class="flow-arrow">&darr; <span>yes: CORRECT &middot; no: keep prediction</span></div>
  <div class="flow-box"><strong>CORRECT</strong><span class="flow-eq">K = P H&#7488; S&#8315;&sup1;<br>x&#770; &larr; x&#770; + K &nu;<br>P &larr; (I&minus;KH)P(I&minus;KH)&#7488; + KRK&#7488;</span></div>
  <div class="flow-arrow">&larr; <span>repeat at 100 Hz</span></div>
</div>
"""


# --------------------------------------------------------------------------
# Inline Markdown
# --------------------------------------------------------------------------

def esc(s):
    return html.escape(s, quote=False)


def _image_sub(m):
    alt, src = m.group(1), m.group(2)
    rel = src[3:] if src.startswith("../") else src
    path = ROOT / rel
    if not path.exists():
        raise SystemExit(f"build_site: missing image {path}")
    b64 = base64.b64encode(path.read_bytes()).decode()
    return (
        f'<img src="data:image/png;base64,{b64}" alt="{esc(alt)}">'
    )


def _link_sub(m):
    text, href = m.group(1), m.group(2)
    if href.startswith("../"):
        href = href[3:]
    base = href.split("#")[0]
    if base.endswith(".md"):
        href = ANCHORS.get(base, "#" + base)
    return f'<a href="{href}">{text}</a>'


def inline(text):
    """Inline Markdown to HTML. Code spans are stashed as placeholders so that
    emphasis may span them (e.g. **use `atan2`**), and $math$ passes through
    untouched for MathJax."""

    code_spans = []

    def _stash(m):
        code_spans.append("<code>" + esc(m.group(1)) + "</code>")
        return f"\x00{len(code_spans) - 1}\x00"

    text = re.sub(r"`([^`]+)`", _stash, text)
    t = esc(text)
    t = re.sub(r"!\[([^\]]*)\]\(([^)\s]+)\)", _image_sub, t)
    t = re.sub(r"\[([^\]]+)\]\(([^)\s]+)\)", _link_sub, t)
    t = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"\*([^*\n]+?)\*", r"<em>\1</em>", t)
    t = re.sub("\x00(\\d+)\x00", lambda m: code_spans[int(m.group(1))], t)
    return t


# --------------------------------------------------------------------------
# Block Markdown
# --------------------------------------------------------------------------

def convert(md_text):
    lines = md_text.split("\n")
    out = []
    para = []

    def flush():
        if para:
            out.append("<p>" + inline(" ".join(para)) + "</p>")
            para.clear()

    i = 0
    while i < len(lines):
        s = lines[i].strip()

        # fenced code
        if s.startswith("```"):
            lang = s[3:].strip()
            i += 1
            code = []
            while i < len(lines) and not lines[i].strip().startswith("```"):
                code.append(lines[i])
                i += 1
            i += 1
            flush()
            if lang == "mermaid":
                out.append(MERMAID_HTML)
            else:
                out.append("<pre><code>" + esc("\n".join(code)) + "</code></pre>")
            continue

        # raw details/summary html
        if s == "<details>":
            flush()
            out.append("<details>")
            i += 1
            continue
        if s.startswith("<summary>") and s.endswith("</summary>"):
            flush()
            out.append(s)
            i += 1
            continue
        if s == "</details>":
            flush()
            out.append("</details>")
            i += 1
            continue

        # drop the per-lesson prev/next nav (single page has the TOC)
        if s.startswith("**Next:**") or s.startswith("**Previous:**"):
            i += 1
            continue

        if s == "":
            flush()
            i += 1
            continue

        if s == "---":
            flush()
            out.append("<hr>")
            i += 1
            continue

        m = re.match(r"^(#{1,4})\s+(.*)$", s)
        if m:
            flush()
            level = len(m.group(1))
            tag = ["h2", "h3", "h4", "h5"][level - 1]
            out.append(f"<{tag}>{inline(m.group(2))}</{tag}>")
            i += 1
            continue

        # standalone image -> figure with caption
        m_img = re.fullmatch(r"!\[([^\]]*)\]\(([^)\s]+)\)", s)
        if m_img:
            flush()
            out.append(
                "<figure>" + inline(s)
                + "<figcaption>" + esc(m_img.group(1)) + "</figcaption></figure>"
            )
            i += 1
            continue

        # blockquote (recursively parsed)
        if s.startswith(">"):
            flush()
            buf = []
            while i < len(lines) and lines[i].strip().startswith(">"):
                buf.append(re.sub(r"^\s*>\s?", "", lines[i]))
                i += 1
            out.append("<blockquote>" + convert("\n".join(buf)) + "</blockquote>")
            continue

        # table
        if s.startswith("|") and i + 1 < len(lines) and re.match(r"^\s*\|[\s:|\-]+\|\s*$", lines[i + 1]):
            flush()
            header = [c.strip() for c in s.strip("|").split("|")]
            i += 2
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
                i += 1
            thead = "<tr>" + "".join(f"<th>{inline(c)}</th>" for c in header) + "</tr>"
            tbody = "".join(
                "<tr>" + "".join(f"<td>{inline(c)}</td>" for c in row) + "</tr>"
                for row in rows
            )
            out.append(f"<table><thead>{thead}</thead><tbody>{tbody}</tbody></table>")
            continue

        # unordered list (with indented continuation lines)
        if re.match(r"^[-*]\s+", s):
            flush()
            items = []
            while i < len(lines):
                raw = lines[i]
                if not re.match(r"^\s*[-*]\s+", raw.strip()) or not raw.strip()[:1] in ("-", "*"):
                    break
                item = re.sub(r"^\s*[-*]\s+", "", raw.strip())
                i += 1
                while (i < len(lines) and lines[i].startswith("  ")
                       and lines[i].strip() != ""
                       and not re.match(r"^\s*[-*]\s+", lines[i].strip())):
                    item += " " + lines[i].strip()
                    i += 1
                items.append("<li>" + inline(item) + "</li>")
            out.append("<ul>" + "".join(items) + "</ul>")
            continue

        # ordered list (with indented continuation lines)
        if re.match(r"^\d+\.\s+", s):
            flush()
            items = []
            while i < len(lines):
                raw = lines[i]
                if not re.match(r"^\s*\d+\.\s+", raw.strip()):
                    break
                item = re.sub(r"^\s*\d+\.\s+", "", raw.strip())
                i += 1
                while (i < len(lines) and lines[i].startswith("  ")
                       and lines[i].strip() != ""
                       and not re.match(r"^\s*\d+\.\s+", lines[i].strip())):
                    item += " " + lines[i].strip()
                    i += 1
                items.append("<li>" + inline(item) + "</li>")
            out.append("<ol>" + "".join(items) + "</ol>")
            continue

        para.append(s)
        i += 1

    flush()
    return "\n".join(out)


# --------------------------------------------------------------------------
# Page assembly
# --------------------------------------------------------------------------

CSS = """
:root{
  --ink:#1c2733; --muted:#5b6b7b; --line:#dde4ea; --bg:#f6f8fa;
  --accent:#2c5f8a; --accent-dark:#1e3a5f;
  --red:#c62828; --blue:#1565c0; --grey:#8a939c;
}
*{box-sizing:border-box}
html{scroll-behavior:smooth}
body{margin:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  color:var(--ink);background:#fff;line-height:1.65;font-size:16.5px}
.hero{background:linear-gradient(135deg,var(--accent-dark),var(--accent));color:#fff;padding:56px 24px 48px}
.hero-inner{max-width:1060px;margin:0 auto}
.hero h1{font-size:2.1em;margin:0 0 12px;line-height:1.25}
.hero p{margin:6px 0;font-size:1.08em;color:#dbe7f2;max-width:760px}
.chips{margin-top:18px;display:flex;gap:10px;flex-wrap:wrap}
.chip{background:rgba(255,255,255,.14);border:1px solid rgba(255,255,255,.3);border-radius:999px;
  padding:4px 14px;font-size:.85em}
.layout{max-width:1060px;margin:0 auto;padding:0 24px;display:flex;gap:40px;align-items:flex-start}
nav.toc{position:sticky;top:24px;width:250px;flex-shrink:0;margin-top:36px;
  border-left:3px solid var(--line);padding-left:16px;font-size:.92em}
nav.toc .toc-h{font-weight:700;color:var(--accent-dark);margin-bottom:8px;font-size:.95em}
nav.toc a{display:block;color:var(--muted);text-decoration:none;padding:4px 0}
nav.toc a:hover{color:var(--accent)}
nav.toc .toc-legend{margin-top:16px;padding-top:12px;border-top:1px dashed var(--line);font-size:.85em}
main{flex:1;min-width:0;padding-bottom:60px}
section.lesson{padding-top:34px;scroll-margin-top:12px}
section.lesson>h2{color:var(--accent-dark);font-size:1.65em;border-bottom:3px solid var(--accent);
  padding-bottom:10px;margin:0 0 8px}
h3{color:var(--accent-dark);margin-top:1.6em}
h4{color:var(--ink);margin-top:1.4em}
a{color:var(--accent)}
code{background:var(--bg);border:1px solid var(--line);border-radius:4px;padding:1px 5px;font-size:.88em;
  font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
pre{background:#0f1c2b;color:#dce8f5;border-radius:8px;padding:14px 16px;overflow-x:auto}
pre code{background:none;border:none;color:inherit;padding:0;font-size:.86em;line-height:1.5}
blockquote{margin:1.2em 0;padding:14px 20px;background:#eef4fa;border-left:5px solid var(--accent);
  border-radius:0 8px 8px 0}
blockquote h4{margin:.2em 0 .5em;color:var(--accent-dark)}
table{border-collapse:collapse;margin:1.2em 0;font-size:.93em;width:100%}
th,td{border:1px solid var(--line);padding:7px 11px;text-align:left;vertical-align:top}
th{background:var(--bg);color:var(--accent-dark)}
tbody tr:nth-child(even){background:var(--bg)}
figure{margin:1.4em 0;text-align:center}
figure img{max-width:100%;border:1px solid var(--line);border-radius:8px;box-shadow:0 2px 10px rgba(20,40,60,.08)}
figcaption{font-size:.86em;color:var(--muted);margin-top:8px;font-style:italic}
details{background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:10px 16px;margin:0.6em 0 1.4em}
details summary{cursor:pointer;font-weight:600;color:var(--accent-dark)}
hr{border:none;border-top:1px solid var(--line);margin:2.2em 0}
.flow{max-width:420px;margin:1.6em auto;display:flex;flex-direction:column;align-items:stretch}
.flow-box{background:#fff;border:2px solid var(--accent);border-radius:10px;padding:10px 16px;text-align:center;
  font-weight:600;color:var(--accent-dark)}
.flow-box.flow-sensor{border-color:var(--grey);color:var(--muted)}
.flow-box.flow-decision{border-style:dashed}
.flow-eq{display:block;font-weight:400;font-family:ui-monospace,Menlo,Consolas,monospace;font-size:.85em;margin-top:4px}
.flow-arrow{text-align:center;color:var(--muted);font-size:.85em;padding:3px 0}
a.top{display:inline-block;margin-top:18px;font-size:.88em;text-decoration:none}
footer{background:var(--accent-dark);color:#b9cde0;padding:28px 24px;font-size:.9em}
footer .hero-inner p{color:#b9cde0}
footer a{color:#fff}
@media(max-width:940px){
  .layout{flex-direction:column}
  nav.toc{position:static;width:100%;border-left:none;border-top:3px solid var(--line);padding:12px 0 0}
  nav.toc a{display:inline-block;padding:3px 12px 3px 0;margin-right:8px}
  .hero h1{font-size:1.6em}
}
"""

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sensor Fusion with Extended Kalman Filters — a practical course</title>
<meta name="description" content="Learn EKF sensor fusion from exact 1D linear foundations to nonlinear 2D estimation, observability limits, and Monte Carlo validation. Free course with GNU Octave demos.">
<script>
MathJax = {{
  tex: {{ inlineMath: [['$','$'], ['\\\\(','\\\\)']], displayMath: [['$$','$$']] }},
  options: {{ skipHtmlTags: ['script','noscript','style','textarea','pre','code'] }}
}};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js" async></script>
<style>{css}</style>
</head>
<body id="top">

<header class="hero">
  <div class="hero-inner">
    <h1>Sensor Fusion with Extended Kalman Filters</h1>
    <p>From exact 1D linear foundations to nonlinear 2D reality &mdash; a hands-on,
       Monte-Carlo-driven course built around three self-contained GNU Octave demos.</p>
    <p>Learn the filter equations, validate them honestly, and discover why a sensor that
       is spectacular in one geometry can be nearly useless in another &mdash; and how
       two beacons turn 67&nbsp;m of error into 9&nbsp;cm.</p>
    <div class="chips">
      <span class="chip">7 lessons</span>
      <span class="chip">3 runnable demos</span>
      <span class="chip">50-run Monte Carlo validation</span>
      <span class="chip">GNU Octave &middot; no toolboxes</span>
      <span class="chip">Exercises with solutions</span>
    </div>
  </div>
</header>

<div class="layout">
<nav class="toc">
  <div class="toc-h">Course content</div>
  {toc}
  <div class="toc-legend">
    <strong>Plot colour language</strong><br>
    <span style="color:var(--grey)">&#9632;</span> individual Monte Carlo runs<br>
    <span style="color:var(--red)">&#9632;</span> filter's claimed 1&sigma;<br>
    <span style="color:var(--blue)">&#9632;</span> empirical 1&sigma; across runs<br>
    Red &asymp; blue: consistent. Blue &gg; red: overconfident.
  </div>
</nav>
<main>
{lessons}
</main>
</div>

<footer>
  <div class="hero-inner">
    <p><strong>SensorFusionDemo</strong> &middot; an open educational repository.
       Demo scripts: <a href="Demo1D.m">Demo1D.m</a> &middot;
       <a href="NonLinear2D.m">NonLinear2D.m</a> &middot;
       <a href="TwoBeacon2D.m">TwoBeacon2D.m</a> &middot;
       License: <a href="LICENSE">MIT</a>.</p>
    <p>Mathematics rendered by MathJax (requires internet); everything else on this page
       is self-contained. Regenerate with <code>python3 tools/build_site.py</code>.</p>
  </div>
</footer>

</body>
</html>
"""


def main():
    toc_items = []
    lesson_html = []
    for i, (fname, sid, title) in enumerate(LESSONS, start=1):
        md = (ROOT / "docs" / fname).read_text()
        # strip the leading "# Lesson N - ..." heading (section header carries it)
        md = re.sub(r"^#\s+.*?\n", "", md)
        body = convert(md)
        lesson_html.append(
            f'<section class="lesson" id="{sid}">\n'
            f"<h2>Lesson {i} &mdash; {title}</h2>\n{body}\n"
            f'<a class="top" href="#top">&uarr; Back to contents</a>\n</section>'
        )
        toc_items.append(f'<a href="#{sid}">Lesson {i} &mdash; {title}</a>')

    page = PAGE.format(toc="\n".join(toc_items), lessons="\n".join(lesson_html), css=CSS)
    out = ROOT / "index.html"
    out.write_text(page)
    n_img = page.count('data:image/png;base64,')
    print(f"built {out} ({out.stat().st_size/1e6:.2f} MB, {n_img} embedded figures)")


if __name__ == "__main__":
    main()
