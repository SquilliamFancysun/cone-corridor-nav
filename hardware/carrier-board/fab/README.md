# Fabrication releases

One directory per board revision, holding **the exact bytes sent to the fab**
plus what was ordered with them.

```
fab/rev-a/
  gerbers.zip     # exactly what was uploaded, byte for byte
  cpl.csv         # pick-and-place, as exported
  order.md        # settings, date, quantity, cost, who ordered it
```

## Why the zip is in git

Gerbers are generated, and generated files usually stay out. This one does not,
for the same reason `deploy.sh` stamps a commit into `VERSION` rather than
trusting that the tree can be reconstructed: **a board revision that cannot be
re-ordered exactly is a board revision you no longer have.**

Re-exporting from a later commit is not the same file. Plugin versions change,
export settings drift, and the difference is invisible until five boards arrive
wrong. The zip is ~200 KB — far under the threshold where this repo pushes
things to a Release.

## Tagging

Tag the commit whose gerbers were ordered:

```sh
git tag board/rev-a
git push origin board/rev-a
```

Same discipline as the `deploy/*` tags, and for the same reason stated in
`docs/hardware-baseline.md`: a sha alone is not enough to get something back,
because GitHub will not serve a bare unadvertised commit and the refusal reads
as if it does not exist.

**Order, then tag, then note the tag in `order.md`.** A rebase after ordering
makes the recorded commit unreachable and the provenance silently breaks.

## order.md

Record what the fab was actually told, not what the spec says it should be —
those diverge, and the fab's copy is what arrived in the box.

- Date ordered, vendor, quantity
- Layers, copper weight, thickness, surface finish
- Assembly: yes/no, which side, any parts hand-placed
- Total cost
- The git tag this was built from
- Anything overridden from the spec, and why
