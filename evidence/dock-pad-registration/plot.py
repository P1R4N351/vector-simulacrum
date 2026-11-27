import csv
import matplotlib.pyplot as plt

with open("sequence.csv") as source:
    rows=[r for r in csv.DictReader(source) if r["face_mm"]=="30"]
fig,(offset,residual)=plt.subplots(1,2,figsize=(10,3.8),layout="constrained")
for r in rows:
    bias=float(r["pitch_bias_deg"])
    five=r["anchors"]=="5"
    x=bias+(0.08 if five else -0.08)
    color="#116c81" if five else "#bb6633"
    label="Glyph + lip" if five else "Glyph only"
    a,b=[float(r[k]) for k in ("last_on_out_hi","first_off_out_hi")]
    offset.plot([x,x],[a,b],c=color,lw=2)
    offset.scatter(x,a,color=color,marker="o",label=label if bias==-2 else None)
    offset.scatter(x,b,color=color,marker="x")
    residual.scatter(x,float(r["held_rear_rms_px"]),color=color)
offset.axhline(9,color="black",ls=":",label="Model edge")
offset.set(ylabel="Outer pad fore–aft offset (mm)",title="Last on (●) to first off (×)")
residual.set(ylabel="Held-out rear coordinate RMS error (px)",title="Prediction from the front hub")
for ax in [offset,residual]:
    ax.set_xlabel("Assumed SDK pitch bias (°)")
    ax.set_xticks([-2,0,2])
offset.legend(fontsize=8)
fig.savefig("sensitivity.png",dpi=155)
