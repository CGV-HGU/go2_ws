import os
import sys
import urllib.request
import subprocess
from pathlib import Path

# RAG 지식 저장소 경로 지정
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
PAPERS_DIR = WORKSPACE_ROOT / ".gemini" / "papers"
CACHE_DIR = WORKSPACE_ROOT / "scratch" / "papers_cache"

def ensure_dependencies():
    """pypdf 패키지가 설치되어 있는지 확인하고 없으면 자동 설치"""
    try:
        import pypdf
    except ImportError:
        print("[*] 'pypdf' package is missing. Installing it automatically via pip...")
        try:
            subprocess.run([sys.executable, "-m", "pip", "install", "pypdf"], check=True)
            print("[🟢] 'pypdf' installed successfully!")
        except Exception as e:
            print(f"[❌] Failed to install pypdf: {e}. Please install it manually with: pip install pypdf")
            sys.exit(1)

def download_arxiv_pdf(arxiv_id: str) -> Path:
    """arXiv에서 PDF를 다운로드하여 캐시에 보관"""
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    pdf_path = CACHE_DIR / f"{arxiv_id}.pdf"
    
    url = f"https://arxiv.org/pdf/{arxiv_id}.pdf"
    print(f"[*] Downloading PDF from {url}...")
    
    # User-Agent를 브라우저처럼 설정하여 차단 방지
    req = urllib.request.Request(
        url, 
        headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'}
    )
    
    with urllib.request.urlopen(req) as response, open(pdf_path, 'wb') as out_file:
        out_file.write(response.read())
        
    print(f"[🟢] PDF downloaded successfully: {pdf_path}")
    return pdf_path

def convert_pdf_to_md(pdf_path: Path, arxiv_id: str) -> Path:
    """PDF 파일의 텍스트를 추출하여 마크다운 파일로 변환 저장"""
    import pypdf
    
    PAPERS_DIR.mkdir(parents=True, exist_ok=True)
    md_path = PAPERS_DIR / f"{arxiv_id}.md"
    
    print(f"[*] Extracting text from {pdf_path}...")
    reader = pypdf.PdfReader(pdf_path)
    
    md_lines = []
    md_lines.append(f"# ArXiv Paper {arxiv_id}")
    md_lines.append(f"Source PDF: {pdf_path.name}\n")
    md_lines.append("---")
    
    for page_num, page in enumerate(reader.pages):
        text = page.extract_text()
        md_lines.append(f"\n## --- Page {page_num + 1} ---\n")
        md_lines.append(text)
        
    md_content = "\n".join(md_lines)
    
    # 텍스트가 비어있을 경우 예외 처리
    if len(md_content.strip()) < 100:
        print("[⚠️ WARNING] Extracted text is extremely short. The PDF might contain scanned images.")
        
    md_path.write_text(md_content, encoding="utf-8")
    print(f"[🟢] Markdown conversion complete: {md_path}")
    return md_path

def main():
    if len(sys.argv) < 2:
        print("Usage: python scratch/archive_paper.py <arxiv_id>")
        print("Example: python scratch/archive_paper.py 2505.00690")
        sys.exit(1)
        
    arxiv_id = sys.argv[1].strip()
    # arXiv 번호 형식 검사 (예: 2505.00690)
    if not arxiv_id.replace(".", "").replace("-", "").isalnum():
        print(f"[❌] Invalid ArXiv ID format: {arxiv_id}")
        sys.exit(1)
        
    ensure_dependencies()
    
    try:
        pdf_path = download_arxiv_pdf(arxiv_id)
        md_path = convert_pdf_to_md(pdf_path, arxiv_id)
        print(f"\n🎉 Successfully archived paper {arxiv_id}!")
        print(f"📍 Saved Location: {md_path}")
    except Exception as e:
        print(f"[❌] Error archiving paper {arxiv_id}: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
