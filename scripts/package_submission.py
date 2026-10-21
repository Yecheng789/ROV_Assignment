#!/usr/bin/env python3

import hashlib
import json
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]


def main():
    dest=ROOT/'deliverables/ROV_Assignment_Project.zip'
    dest.parent.mkdir(exist_ok=True)
    folders=['rov_survey','scripts','config','tests','resource','worlds','vendor/bluerov2_gz',
             'docs','videos','results/current','results/calm','results/strong_current',
             'results/report_evidence','results/video_run_20261021']
    files=['README.md','Assignment1.pdf','THIRD_PARTY.rst','package.xml','setup.py','setup.cfg',
           'pyproject.toml','requirements.txt','results/summary.json','results/validation.json']
    selected=set()
    for folder in folders:
        selected.update(p for p in (ROOT/folder).rglob('*') if p.is_file())
    selected.update(ROOT/name for name in files if (ROOT/name).is_file())
    selected={p for p in selected if '__pycache__' not in p.parts
              and not any(part in ('gz_home','ros_logs','.mplconfig','word_preview','revisions','review') for part in p.relative_to(ROOT).parts)
              and p.suffix not in ('.pyc',)}
    manifest={'files':{},'exclusions':'Local conversion tools, build products, review inputs, archived report revisions, previews and runtime caches.'}
    with ZipFile(dest,'w',ZIP_DEFLATED,compresslevel=6) as archive:
        for path in sorted(selected):
            relative=path.relative_to(ROOT).as_posix()
            archive.write(path,'ROV_Assignment/'+relative)
            manifest['files'][relative]=hashlib.sha256(path.read_bytes()).hexdigest()
        archive.writestr('ROV_Assignment/SUBMISSION_MANIFEST.json',json.dumps(manifest,indent=2)+'\n')
    with ZipFile(dest) as archive:
        assert archive.testzip() is None
        for name in ['docs/report.pdf','docs/report.docx','videos/ROV_Survey_Full_Run.mp4',
                     'results/video_run_20261021/status.json','rov_survey/node.py','config/bridge.yaml']:
            assert 'ROV_Assignment/'+name in archive.namelist()
    (dest.parent/'package_manifest.json').write_text(json.dumps({'archive':dest.name,
        'size_bytes':dest.stat().st_size,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),
        'file_count':len(selected),'archive_crc_check_passed':True},indent=2)+'\n')
    print(f'{dest} ({dest.stat().st_size/1024/1024:.1f} MiB; {len(selected)} files)')


if __name__=='__main__': main()
