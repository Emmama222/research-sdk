from pathlib import Path
from research_sdk.world.map.voronoi.voronoi_generator import generate_bounded_voronoi_map
m = generate_bounded_voronoi_map(obstacles=())
x0,x1,y0,y1=m.field_bounds_mm
w,h=1000,720
scale=min(880/(x1-x0),560/(y1-y0))
def p(x,y): return (500+x*scale,365-y*scale)
s=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="720" viewBox="0 0 1000 720">','<rect width="1000" height="720" fill="#f8fafc"/>',f'<text x="500" y="35" text-anchor="middle" font-family="sans-serif" font-size="24">Empty-field Voronoi · {len(m.virtual_sites_mm)} virtual sites · 0 obstacles</text>']
x,y=p(x0,y1)
s.append(f'<rect x="{x}" y="{y}" width="{(x1-x0)*scale}" height="{(y1-y0)*scale}" fill="white" stroke="#94a3b8" stroke-dasharray="6 4"/>')
for c in m.cells:
    pts=' '.join(f'{a},{b}' for a,b in (p(*v) for v in c.polygon_mm))
    s.append(f'<polygon points="{pts}" fill="none" stroke="#93c5fd" stroke-width="1.5"/>')
nodes={n.id:n for n in m.nodes}
for e in m.edges:
    a,b=nodes[e.start_id],nodes[e.end_id]
    ax,ay=p(a.x,a.y); bx,by=p(b.x,b.y)
    s.append(f'<line x1="{ax}" y1="{ay}" x2="{bx}" y2="{by}" stroke="#2563eb" stroke-width="2.5"/>')
for x,y in m.virtual_sites_mm:
    a,b=p(x,y)
    s.append(f'<circle cx="{a}" cy="{b}" r="6" fill="#e94560"/>')
s.append(f'<text x="500" y="675" text-anchor="middle" font-family="sans-serif" font-size="16">Pink: virtual sites · Blue: navigation edges · Dashed: field boundary</text>')
s.append(f'<text x="500" y="703" text-anchor="middle" font-family="sans-serif" font-size="14" fill="#64748b">Field {(x1-x0)/1000:g} × {(y1-y0)/1000:g} m · Current display defaults · {len(m.edges)} edges</text></svg>')
out=Path('results/voronoi_empty_virtual_sites.svg');out.parent.mkdir(exist_ok=True);out.write_text('\n'.join(s),encoding='utf-8')
print(out.resolve()); print(f'{len(m.virtual_sites_mm)} sites, {len(m.obstacles)} obstacles, {len(m.edges)} edges')
