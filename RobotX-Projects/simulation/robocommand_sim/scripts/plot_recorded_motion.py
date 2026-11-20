#!/usr/bin/env python3
"""Plot actual UAV/USV Gazebo tracks from a RoboCommand recorder folder."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import yaml


COLORS = {"green": "#17843b", "red": "#d93a32", "blue": "#225bd9", "black": "#25292e"}
UAV = "#146da8"
USV = "#d87c10"


def timestamp(value: str) -> float:
    return datetime.fromisoformat(value).timestamp()


def numeric(rows: list[dict], key: str) -> np.ndarray:
    return np.array([float(row[key]) if row[key] else math.nan for row in rows])


def first_event(events: list[dict], topic: str) -> float | None:
    for event in events:
        if event["topic"] == topic:
            return timestamp(event["received_utc"])
    return None


def xy_at(t: np.ndarray, x: np.ndarray, y: np.ndarray, when: float) -> tuple[float,float]:
    candidates = np.where(np.isfinite(x) & np.isfinite(y))[0]
    if not len(candidates):
        return math.nan, math.nan
    index = candidates[np.argmin(abs(t[candidates]-when))]
    return float(x[index]), float(y[index])


def arrows(ax, x: np.ndarray, y: np.ndarray, t: np.ndarray, active: np.ndarray,
           color: str, zoom: bool) -> None:
    indices = np.where(active & np.isfinite(x) & np.isfinite(y))[0]
    if len(indices) < 20:
        return
    for fraction in (.2, .4, .6, .8):
        middle = indices[min(len(indices)-1, int(fraction*len(indices)))]
        before = max(indices[0], middle-12)
        after = min(indices[-1], middle+12)
        dx, dy = x[after]-x[before], y[after]-y[before]
        if not np.isfinite(dx+dy) or math.hypot(dx,dy) < .7:
            continue
        mx,my = x[middle],y[middle]
        if zoom and not (-37 <= mx <= 22 and -45 <= my <= 31):
            continue
        norm = math.hypot(dx,dy)
        ax.annotate("", xy=(mx+1.1*dx/norm,my+1.1*dy/norm),
                    xytext=(mx-1.1*dx/norm,my-1.1*dy/norm),
                    arrowprops={"arrowstyle":"-|>","color":color,"lw":1.7,
                                "mutation_scale":15},zorder=9)


def plot(recording: Path, output: Path, animate: bool = False) -> dict:
    recording = recording.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (recording/"telemetry.csv").open(newline="",encoding="utf-8") as file:
        rows=list(csv.DictReader(file))
    events=[json.loads(line) for line in (recording/"events.jsonl").read_text(encoding="utf-8").splitlines() if line]
    course=yaml.safe_load((recording/"task1_world.yaml").read_text(encoding="utf-8"))
    recorded=json.loads((recording/"summary.json").read_text(encoding="utf-8"))
    if not rows:
        raise ValueError("telemetry.csv has no samples")
    wall=numeric(rows,"elapsed_s")
    start_utc=timestamp(rows[0]["wall_time_utc"])
    go_utc=first_event(events,"/robocommand/team/go")
    stop_utc=first_event(events,"/robocommand/control/stop")
    go=(go_utc-start_utc) if go_utc else float(wall[0])
    stop=(stop_utc-start_utc) if stop_utc else float(wall[-1])
    if stop <= go:
        raise ValueError("STOP precedes GO in this recording")
    t=wall-go
    active=(wall>=go)&(wall<=stop)
    pre=wall<go
    post=wall>stop
    ux,uy,uz=(numeric(rows,key) for key in ("uav_x_m","uav_y_m","uav_z_m"))
    bx,by,bz=(numeric(rows,key) for key in ("usv_x_m","usv_y_m","usv_z_m"))
    route_event=next((event for event in events if event["topic"]=="/usv/mission/sim_route"
                      and event["message"].get("route")),None)
    route=route_event["message"]["route"] if route_event else []
    route_t=(timestamp(route_event["received_utc"])-start_utc-go) if route_event else None
    if route:
        rx=np.array([float(point["x"]) for point in route])
        ry=np.array([float(point["y"]) for point in route])
    polygon=np.array(course["survey_polygon_xy"],dtype=float)
    survey=np.array(course["survey_waypoints_xy"],dtype=float)

    plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":10,
                         "axes.spines.top":False, "axes.spines.right":False})
    fig,axes=plt.subplots(1,2,figsize=(17,8.5),gridspec_kw={"width_ratios":[1.05,1]})
    for zoom,ax in enumerate(axes):
        closed=np.vstack((polygon,polygon[0]))
        ax.plot(closed[:,0],closed[:,1],color="#a4aeb9",lw=1.2,ls="--",
                label="Survey area" if zoom==0 else None,zorder=1)
        ax.plot(survey[:,0],survey[:,1],color=UAV,lw=1,ls=":",alpha=.65,
                label="UAV survey waypoints" if zoom==0 else None,zorder=1)
        if route:
            ax.plot(rx,ry,color="#555c66",lw=1.5,ls="--",alpha=.85,
                    label="Planned boat route" if zoom==0 else None,zorder=2)
            if zoom:
                for index in (0,1,2):
                    ax.annotate(f"P{index+1}",(rx[index],ry[index]),xytext=(4,4),
                                textcoords="offset points",color="#4f5964",fontsize=8)
        for buoy in course["ground_truth"]:
            color=COLORS[buoy["colour"]]
            is_blink=buoy.get("lit")=="blinking"
            ax.scatter(buoy["x"],buoy["y"],s=94 if zoom else 62,
                       color=color,edgecolor="#f1bc42" if is_blink else "white",
                       linewidth=2.1 if is_blink else 1,zorder=8)
            if zoom:
                ax.annotate(buoy["id"],(buoy["x"],buoy["y"]),xytext=(4,5),
                            textcoords="offset points",fontsize=7,color="#31363d")
        for x,y,color,name in ((ux,uy,UAV,"Drone recorded"),(bx,by,USV,"Boat recorded")):
            ax.plot(x[pre],y[pre],color=color,lw=.7,alpha=.24,zorder=3)
            ax.plot(x[active],y[active],color=color,lw=2.5,
                    label=name if zoom==0 else None,zorder=5)
            ax.plot(x[post],y[post],color=color,lw=1,ls=":",alpha=.55,zorder=3)
            start=xy_at(wall,x,y,go)
            end=xy_at(wall,x,y,stop)
            ax.scatter(*start,marker="o",s=85,facecolor="white",edgecolor=color,
                       linewidth=2.3,zorder=10)
            ax.scatter(*end,marker="X",s=96,color=color,edgecolor="white",linewidth=.5,zorder=10)
            arrows(ax,x,y,wall,active,color,bool(zoom))
            if zoom:
                ax.annotate(f"{name.split()[0]} GO",start,xytext=(6,5),
                            textcoords="offset points",fontsize=8,color=color)
                ax.annotate(f"{name.split()[0]} STOP",end,xytext=(6,-12),
                            textcoords="offset points",fontsize=8,color=color)
        ax.set(xlabel="Gazebo world X (m)",ylabel="Gazebo world Y (m)")
        ax.set_aspect("equal",adjustable="box")
        ax.grid(alpha=.2)
        if zoom:
            ax.set(xlim=(-38,23),ylim=(-46,31),title="Buoy course detail")
        else:
            ax.set(xlim=(-43,45),ylim=(-63,83),title="Full recorded area (GO to STOP highlighted)")
    fig.legend(loc="lower center",ncol=4,bbox_to_anchor=(.5,.025),frameon=False)
    fig.suptitle(f"Drone and boat 2D motion | {recorded['tier'].capitalize()} run {recorded['run_id']}\n"
                 f"GO to STOP: {stop-go:.1f} s | circles = GO, X = STOP, arrows = travel direction",
                 fontsize=15,fontweight="bold")
    fig.subplots_adjust(left=.055,right=.985,top=.86,bottom=.15,wspace=.22)
    fig.savefig(output/"drone_boat_2d_map.png",dpi=190,facecolor="white")
    fig.savefig(output/"drone_boat_2d_map.pdf",facecolor="white")
    plt.close(fig)

    fig,axs=plt.subplots(4,1,figsize=(13,10),sharex=True)
    valid=active
    for ax in axs:
        ax.grid(alpha=.23)
        if route_t is not None and 0<=route_t<=stop-go:
            ax.axvline(route_t,color="#666",lw=.9,ls="--",alpha=.8)
    axs[0].plot(t[valid],ux[valid],color=UAV,label="Drone X")
    axs[0].plot(t[valid],bx[valid],color=USV,label="Boat X")
    axs[0].set(ylabel="World X (m)",title="East/west motion")
    axs[1].plot(t[valid],uy[valid],color=UAV,label="Drone Y")
    axs[1].plot(t[valid],by[valid],color=USV,label="Boat Y")
    axs[1].axhline(min(polygon[:,1]),color="#aa5555",ls=":",lw=.8,
                   label="South survey boundary")
    axs[1].set(ylabel="World Y (m)",title="North/south motion")
    axs[2].plot(t[valid],uz[valid],color=UAV,label="Drone height")
    axs[2].plot(t[valid],bz[valid],color=USV,label="Boat height")
    axs[2].set(ylabel="World Z (m)",title="Height")
    route_index=numeric(rows,"route_index")
    thrust=np.vstack([numeric(rows,"thrust_"+name)
                      for name in ("port","starboard","bow","stern")])
    peak=np.max(np.where(np.isfinite(thrust),np.abs(thrust),0),axis=0)
    peak[~np.isfinite(thrust).any(axis=0)]=np.nan
    axs[3].step(t[valid],route_index[valid],where="post",color="#39434e",label="Route index")
    twin=axs[3].twinx()
    twin.plot(t[valid],peak[valid],color=USV,alpha=.6,lw=.8,label="Largest |thrust request|")
    twin.set_ylabel("Gazebo thrust request")
    axs[3].set(ylabel="Route index",xlabel="Seconds after GO",title="Boat route progress and requested thrust")
    for ax in axs[:3]:
        ax.legend(loc="best",fontsize=8)
    axs[3].legend(loc="upper left",fontsize=8)
    twin.legend(loc="upper right",fontsize=8)
    if route_t is not None:
        axs[0].annotate("Route first published",(route_t,axs[0].get_ylim()[1]),
                        xytext=(5,-12),textcoords="offset points",fontsize=8,color="#555")
    fig.suptitle("Recorded motion over time | GO to STOP",fontsize=15,fontweight="bold")
    fig.tight_layout(rect=(0,0,1,.96))
    fig.savefig(output/"motion_over_time.png",dpi=190,facecolor="white")
    fig.savefig(output/"motion_over_time.pdf",facecolor="white")
    plt.close(fig)

    figures=["drone_boat_2d_map.png","motion_over_time.png"]
    if animate:
        fig,ax=plt.subplots(figsize=(7.5,9))
        ax.plot(closed[:,0],closed[:,1],color="#a4aeb9",lw=1.1,ls="--")
        if route:
            ax.plot(rx,ry,color="#555c66",lw=1.2,ls="--",alpha=.8,label="Planned boat route")
        ax.plot(survey[:,0],survey[:,1],color=UAV,lw=.8,ls=":",alpha=.6)
        for buoy in course["ground_truth"]:
            ax.scatter(buoy["x"],buoy["y"],color=COLORS[buoy["colour"]],s=62,
                       edgecolor="#f1bc42" if buoy.get("lit")=="blinking" else "white",
                       linewidth=1.5,zorder=5)
        uav_line,=ax.plot([],[],color=UAV,lw=2.2,label="Drone")
        usv_line,=ax.plot([],[],color=USV,lw=2.2,label="Boat")
        uav_dot,=ax.plot([],[],"o",color=UAV,markersize=9)
        usv_dot,=ax.plot([],[],"o",color=USV,markersize=9)
        clock=ax.text(.02,.98,"",transform=ax.transAxes,va="top",ha="left",
                      fontsize=11,bbox={"facecolor":"white","edgecolor":"none","alpha":.85})
        ax.set(xlim=(-43,45),ylim=(-63,83),xlabel="Gazebo world X (m)",
               ylabel="Gazebo world Y (m)",title="Drone and boat moving through the recorded run")
        ax.set_aspect("equal",adjustable="box")
        ax.grid(alpha=.2)
        ax.legend(loc="lower right",fontsize=9)
        fig.tight_layout()
        frames=np.linspace(go,stop,110)
        begin=int(np.searchsorted(wall,go))
        def update(frame):
            index=min(len(wall)-1,int(np.searchsorted(wall,frame)))
            uav_line.set_data(ux[begin:index+1],uy[begin:index+1])
            usv_line.set_data(bx[begin:index+1],by[begin:index+1])
            uav_dot.set_data([ux[index]],[uy[index]])
            usv_dot.set_data([bx[index]],[by[index]])
            clock.set_text(f"{frame-go:5.1f} s after GO | playback accelerated")
            return uav_line,usv_line,uav_dot,usv_dot,clock
        animation=FuncAnimation(fig,update,frames=frames,interval=125,blit=False)
        animation.save(output/"drone_boat_motion.gif",writer=PillowWriter(fps=8),dpi=95)
        plt.close(fig)
        figures.append("drone_boat_motion.gif")

    result={"recording":str(recording),"run_id":recorded["run_id"],
            "tier":recorded["tier"],"go_to_stop_s":round(stop-go,2),
            "planned_route_points":len(route),
            "max_logged_route_index":int(np.nanmax(route_index)) if np.isfinite(route_index).any() else None,
            "drone_go_xy_m":list(xy_at(wall,ux,uy,go)),
            "drone_stop_xy_m":list(xy_at(wall,ux,uy,stop)),
            "boat_go_xy_m":list(xy_at(wall,bx,by,go)),
            "boat_stop_xy_m":list(xy_at(wall,bx,by,stop)),
            "boat_min_y_during_go_to_stop_m":round(float(np.nanmin(by[valid])),2),
            "south_survey_boundary_y_m":float(min(polygon[:,1])),
            "figures":figures}
    (output/"plot_summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording",type=Path)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--animate",action="store_true",help="Also save a short animated 2D GIF")
    args=parser.parse_args()
    output=args.output or args.recording/"plots"
    print(json.dumps(plot(args.recording,output,args.animate),indent=2))


if __name__=="__main__":
    main()
