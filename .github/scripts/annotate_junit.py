"""Turn pytest JUnit XML failures into GitHub Actions error annotations.

Annotations show on the run page and through the public API, so a failed upgrade guard is
readable without opening (or having permission to download) the full job log.
"""

import sys
import xml.etree.ElementTree as ET

for case in ET.parse(sys.argv[1]).iter("testcase"):
    for problem in [*case.iter("failure"), *case.iter("error")]:
        text = (problem.get("message") or "") + "\n" + (problem.text or "")[-3000:]
        text = text.replace("%", "%25").replace("\r", "").replace("\n", "%0A")
        print(f"::error title={case.get('classname')}.{case.get('name')}::{text}")
