# Source snapshots

This workspace includes the source trees below as ordinary tracked files so
cloning KSP-GCODE includes the implementation used for the G-code audit.
Local nested `.git` directories are retained on the author's machine but are
not included in this repository. File executable modes follow the nested Git
indexes, rather than permission changes introduced by copying the PC snapshot.

| Directory | Upstream | Base revision |
| --- | --- | --- |
| kiauh | https://github.com/dw-0/kiauh | 372bab8847dc69fc5f557c174a623a9e307a7339 |
| klipper | https://github.com/Klipper3d/klipper | baf188bd62bb9c82775d679cf0db30a72f9e9173 |
| klipper-led_effect | https://github.com/julianschill/klipper-led_effect | a558cb7c88430f00fd9be8f534f9aeb22bd8f3e9 |
| mainsail-config | https://github.com/mainsail-crew/mainsail-config | ff3869a621db17ce3ef660adbbd3fa321995ac42 |
| moonraker | https://github.com/Arksine/moonraker | 960e933837413b4b365074580aeb14d04437204e |

Original licenses remain in each source tree.

The local Moonraker database contains authentication credentials and is excluded
from public Git history. Audit aggregates and selected job metadata are available
in `Working Folder_Adachi/audit/evidence.json`. Re-running the full `verify.py`
requires the locally retained database. The candidate regression checks in
`validate_followup.py` do not require the database.
