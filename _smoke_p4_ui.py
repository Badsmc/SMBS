import os, sys, py_compile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.abspath(__file__))

print("=== syntax check ===")
for rel in ("ui/task_panel.py", "ui/simulation_viewer.py",
            "InitGui.py", "freecad_sequence_debug.py"):
    p = os.path.join(ROOT, rel)
    if not os.path.isfile(p):
        print("  SKIP:", rel); continue
    try:
        py_compile.compile(p, doraise=True)
        print("  {} ... OK".format(rel))
    except py_compile.PyCompileError as e:
        print("  {} ... FAIL: {}".format(rel, e))

print()
print("=== audit import ===")
try:
    import freecad_sequence_debug as dbg
    a = dbg.ProjectAuditor()
    rep = a.run(scene=False)
    print("  version:", dbg.MODULE_VERSION)
    print("  status: ", rep.status())
    print("  counts: ", rep.counts())
    print("  files:  ", rep.metrics.get("python_files_scanned"))
    for i, f in enumerate(rep.findings, 1):
        print("  [{}] {} / {}".format(i, f.severity, f.audit_id))
except Exception as e:
    import traceback; traceback.print_exc()

print()
print("DONE")
