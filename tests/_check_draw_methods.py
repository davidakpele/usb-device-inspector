"""Quick AST checker — verifies every _draw* method that uses _qpt/_proj
has cx, cy, cc, sc, z in its parameter list."""
import ast, sys, pathlib

src  = pathlib.Path("app/ui/drone_3d_view.py").read_text(encoding="utf-8")
tree = ast.parse(src)
print("AST parse: OK")

cls = next(n for n in ast.walk(tree)
           if isinstance(n, ast.ClassDef) and n.name == "Drone3DWidget")

issues = []
ok     = []

for node in ast.walk(cls):
    if not isinstance(node, ast.FunctionDef):
        continue
    name = node.name
    if not (name.startswith("_draw") or name == "_rotors"):
        continue
    params    = [a.arg for a in node.args.args]
    body_src  = ast.unparse(node)
    uses_proj = "_qpt" in body_src or "_proj" in body_src
    if uses_proj and ("cx" not in params or "z" not in params):
        issues.append(
            f"  PROBLEM: {name}() uses projection but params are {params[1:]}")
    else:
        ok.append(f"  OK  {name}  params={params[1:]}")

for line in ok:
    print(line)

if issues:
    print()
    print("ISSUES:")
    for i in issues:
        print(i)
    sys.exit(1)
else:
    print()
    print("All draw methods have correct signatures.")
