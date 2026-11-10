"""Builds docs/index.html from docs/guide.md.

@file path   embeds a file from the repository, with line numbers
@out name    embeds docs/outputs/name.txt as terminal output
"""

import html
import re
from pathlib import Path

import markdown

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"

PY_KW = r"\b(import|from|for|in|if|elif|else|while|def|return|break|continue|not|and|or|is|None|True|False|with|as|class|raise|try|except|pass)\b"


def highlight(line: str, lang: str) -> str:
    if lang == "py":
        comment = ""
        match = re.search(r"#.*$", line)
        if match and line[: match.start()].count('"') % 2 == 0:
            line, comment = line[: match.start()], match.group(0)
        out = []
        for i, part in enumerate(re.split(r'("""[^"]*"""|"[^"]*"|\'[^\']*\')', line)):
            if i % 2:
                out.append(f'<span class="s">{html.escape(part)}</span>')
            else:
                escaped = html.escape(part)
                escaped = re.sub(PY_KW, r'<span class="k">\1</span>', escaped)
                out.append(escaped)
        if comment:
            out.append(f'<span class="c">{html.escape(comment)}</span>')
        return "".join(out)
    if lang in ("yml", "toml"):
        if line.strip().startswith("#"):
            return f'<span class="c">{html.escape(line)}</span>'
        comment = ""
        if " #" in line:
            line, comment = line.split(" #", 1)
            comment = " #" + comment
        match = re.match(r"^(\s*-?\s*)([\w\-\.]+)(:)(.*)$", line)
        if match:
            value = html.escape(match.group(4))
            if "/" in match.group(4):
                value = f'<span class="s">{value}</span>'
            body = f'{html.escape(match.group(1))}<span class="k">{html.escape(match.group(2))}</span>:{value}'
        else:
            body = html.escape(line)
        return body + (f'<span class="c">{html.escape(comment)}</span>' if comment else "")
    return html.escape(line)


def file_block(path: str) -> str:
    source = (ROOT / path).read_text().rstrip("\n").split("\n")
    lang = path.rsplit(".", 1)[-1].replace("yaml", "yml")
    rows = "".join(
        f'<span class="ln">{n}</span><span class="code">{highlight(line, lang) or " "}</span>\n'
        for n, line in enumerate(source, 1)
    )
    return (
        f'<figure class="file"><figcaption><span class="path">{html.escape(path)}</span>'
        f'<span class="lines">{len(source)} lines</span></figcaption>'
        f'<pre class="numbered">{rows}</pre></figure>'
    )


def output_block(name: str) -> str:
    text = (DOCS / "outputs" / f"{name}.txt").read_text().rstrip("\n")
    lines = []
    for line in text.split("\n"):
        escaped = html.escape(line)
        if line.startswith("$ "):
            escaped = f'<span class="prompt">$</span> <span class="cmd">{html.escape(line[2:])}</span>'
        lines.append(escaped)
    return (
        '<figure class="terminal"><figcaption>real output</figcaption><pre>'
        + "\n".join(lines)
        + "</pre></figure>"
    )


def build() -> None:
    source = (DOCS / "guide.md").read_text()
    blocks: dict[str, str] = {}

    def stash(html_text: str) -> str:
        key = f"BLOCK{len(blocks)}X"
        blocks[key] = html_text
        return f"\n\n{key}\n\n"

    source = re.sub(r"^@file (\S+)\s*$", lambda m: stash(file_block(m.group(1))), source, flags=re.M)
    source = re.sub(r"^@out (\S+)\s*$", lambda m: stash(output_block(m.group(1))), source, flags=re.M)
    body = markdown.markdown(source, extensions=["tables", "fenced_code", "attr_list", "toc"])
    for key, value in blocks.items():
        body = body.replace(f"<p>{key}</p>", value)

    nav = []
    for match in re.finditer(r'<h([12]) id="([^"]+)">(.*?)</h\1>', body):
        level, anchor, title = match.groups()
        title = re.sub(r"<.*?>", "", title)
        nav.append(f'<a class="l{level}" href="#{anchor}">{title}</a>')

    template = (DOCS / "template.html").read_text()
    page = template.replace("{{NAV}}", "\n".join(nav)).replace("{{BODY}}", body)
    (DOCS / "index.html").write_text(page)
    print(f"wrote docs/index.html, {len(page) // 1024} KB, {len(nav)} nav entries")


if __name__ == "__main__":
    build()
