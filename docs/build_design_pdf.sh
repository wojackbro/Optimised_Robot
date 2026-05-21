#!/usr/bin/env bash
# Build ISRLAB design documentation: Word (.docx) then PDF.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DOCS="$ROOT/docs"
MD="$DOCS/DESIGN_DOCUMENTATION.md"
BUILD="$DOCS/.build"
DOCX="$DOCS/ISRLAB_Design_Documentation.docx"
HTML="$BUILD/design.html"
PDF="$DOCS/ISRLAB_Design_Documentation.pdf"

CHROME="${CHROME:-/Applications/Google Chrome.app/Contents/MacOS/Google Chrome}"

if [[ ! -f "$MD" ]]; then
  echo "Missing: $MD" >&2
  exit 1
fi

if ! command -v pandoc >/dev/null 2>&1; then
  echo "pandoc is required. Install: brew install pandoc" >&2
  exit 1
fi

mkdir -p "$BUILD"

echo "Building Word document..."
pandoc "$MD" \
  -o "$DOCX" \
  --from=gfm \
  --metadata title="Optimised Robot — Design Documentation" \
  --resource-path="$DOCS:$ROOT" \
  --toc \
  --toc-depth=2

echo "DOCX written: $DOCX"

echo "Building HTML for PDF..."
pandoc "$MD" \
  -o "$HTML" \
  --standalone \
  --from=gfm \
  --metadata title="Optimised Robot — Design Documentation" \
  --css="$DOCS/pdf_style.css" \
  --resource-path="$DOCS:$ROOT" \
  --toc \
  --toc-depth=2

if [[ ! -x "$CHROME" ]]; then
  echo "Chrome not found at: $CHROME" >&2
  echo "Set CHROME to your browser binary, or install Google Chrome." >&2
  echo "HTML built at: $HTML (open in browser → Print to PDF)" >&2
  exit 1
fi

echo "Building PDF..."
"$CHROME" \
  --headless=new \
  --disable-gpu \
  --no-pdf-header-footer \
  --print-to-pdf="$PDF" \
  "file://$HTML"

echo "PDF written: $PDF"
