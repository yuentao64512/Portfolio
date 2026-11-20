# OptiPrime RobotX 2026 - MASTER_SCRIPTS (package: optiprimex; the laptop bench, the tests, the host launchers)
#
#   make venv      create .venv (Python 3.10+) and install the package editable + test deps
#   make test      the whole suite on 6 processes, a file per process (~3 min; every WP version + before a Jetson deploy)
#   make live      quick + the live wire / safety tests, without the 4 long live missions (~1-2 min; launchers, wire, usv_ui, OCS)
#   make quick     unit + every mission once in the fast SIL, no brokers (< 1 min; every change, side fixes included)
#   make day       v0.1.86 (WP42 §2.4): quick on 6 processes + the live wire subset (DAY_LIVE) - between runs on a competition day
#   make test-serial  the whole suite one test at a time (~10 min; the old way, to rule out the parallel run)
#   make unit      unit tests only (no brokers needed)
#   make bench     start the laptop bench in tmux (brokers, fake FCU, mission manager, fake RoboCommand, OCS, loggers)
#   make sim       our simulator (WP10): the bench + the rendered world in its own window (needs make venv-sim once)
#   make venv-sim  the simulator's extra dependencies (numpy, Pillow) into .venv
#   make stop      stop it (make stop WIPE=1 also clears broker persistence)
#   make logs      tail the current run's logs
#   make clean     remove caches and logs (keeps the venv)
#   make help      this list

PYTHON ?= python3.10
VENV   := .venv
PY     := $(VENV)/bin/python
PIP    := $(VENV)/bin/pip

.PHONY: help venv venv-sim sim test test-serial live unit quick day integration bench stop logs clean distclean check-mosquitto

help:
	@sed -n '3,13p' Makefile | sed 's/^#   //'

venv: $(VENV)/.ready
$(VENV)/.ready: pyproject.toml requirements.txt
	@command -v $(PYTHON) >/dev/null || { echo "$(PYTHON) not found (brew install python@3.10, or PYTHON=python3.11 make venv)"; exit 1; }
	$(PYTHON) -m venv $(VENV)
	$(PIP) install --upgrade pip wheel
	$(PIP) install -r requirements.txt
	$(PIP) install -e .
	@touch $@
	@echo "venv ready: source $(VENV)/bin/activate"

check-mosquitto:
	@command -v mosquitto >/dev/null || [ -x /opt/homebrew/sbin/mosquitto ] || [ -x /usr/local/sbin/mosquitto ] || \
	  { echo "mosquitto not found: brew install mosquitto"; exit 1; }

# v0.1.39.3: three tiers (Dmitrii 2026-11-05). The parallel run keeps each test file on one process (--dist loadfile): the live
# modules share a stack and some tests build on the one before (test_integration_sil1's declaration_seq) - split, they fail.
JOBS ?= 6
LONG_LIVE = m1t1sim_live or m0t4sim_live or m3t1sim_live or dockto_live_then_cannon
test: venv check-mosquitto
	$(PY) -m pytest -n $(JOBS) --dist loadfile

test-serial: venv check-mosquitto
	$(PY) -m pytest

live: venv check-mosquitto
	$(PY) -m pytest -n $(JOBS) --dist loadfile -k "not ($(LONG_LIVE))"

# v0.1.86 (WP42 §2.4, Dmitrii: "reshuffle which tests we make between builds, so we don't wait during the day"): the day rule's tier -
# QUICK on 6 processes + the live wire / safety files a launcher, broker, usv_ui, OCS or mission-manager change touches (the real usv_ui
# and the OCS backend on live brokers, the integration SIL). Prints its runtime.
DAY_LIVE = tests/test_usv_ui.py tests/test_v0143_backend.py tests/test_integration_sil1.py tests/test_v0164.py
day: venv check-mosquitto
	@t0=$$(date +%s); $(PY) -m pytest -n $(JOBS) --dist loadfile $(QUICK) $(DAY_LIVE) -k "not judge_web and not against_the_protobuf_fake and not real_wire and not dir_source and not twenty_scout_runs and not ($(LONG_LIVE))" \
	  && echo "make day: $$(( $$(date +%s) - t0 )) s"

unit: venv
	$(PY) -m pytest tests/test_schemas_and_envelope.py tests/test_vehicle_fsm_table.py tests/test_frames_health_adapter.py \
	  tests/test_run_context_tasks.py tests/test_ocs_units.py

# v0.1.26 (WP7): the unit tests and every mission in the fast SIL (optiprimex/sim/fast_sil.py) - nothing that needs a broker
QUICK = tests/test_schemas_and_envelope.py tests/test_vehicle_fsm_table.py tests/test_frames_health_adapter.py \
	tests/test_run_context_tasks.py tests/test_ocs_units.py tests/test_perception_map.py tests/test_playbook_tui.py \
	tests/test_quick_missions.py tests/test_stage_stepping.py tests/test_t1_sim.py tests/test_t3_sim.py tests/test_sim_pool.py \
	tests/test_protobuf_judge.py tests/test_cam_stream.py tests/test_scout.py tests/test_guard.py tests/test_station_keep.py tests/test_map_obstacles.py tests/test_v0182.py tests/test_v018302.py tests/test_v018304.py tests/test_v018305.py tests/test_v018405.py tests/test_svo2_mp4.py tests/test_launcher_tk.py tests/test_ocs_tk_model.py tests/test_ocs_tk_net.py tests/test_ocs_tk_toasts.py tests/test_ocs_tk_smoke.py tests/test_ocs_tk_map.py tests/test_ocs_tk_hosts.py tests/test_ocs_tk_pull_logs.py tests/test_v0185.py tests/test_v0186.py tests/test_ocs_tk_panels.py tests/test_v018601.py tests/test_v018604.py \
	tests/test_box_inspection.py tests/test_box_perception_contract.py tests/test_box_task_integration.py tests/test_box_wp8.py tests/test_mode_log.py tests/test_ocs_web.py tests/test_ocs_web_settings.py tests/test_wp8b.py tests/test_wp8c_cv.py tests/test_pool_fixes.py tests/test_wp8c.py tests/test_wp9a.py tests/test_frame_compare.py tests/test_sim_wp10.py tests/test_wp10b.py tests/test_wp10g.py tests/test_wp10c.py tests/test_wp10e.py tests/test_wp10d.py tests/test_launcher.py tests/test_wp10h.py tests/test_make_tiers.py tests/test_wp9b.py tests/test_wp9c.py tests/test_wp9d.py tests/test_v0143.py tests/test_v0144.py tests/test_v0145.py tests/test_v0146.py tests/test_v01461.py tests/test_v0147.py tests/test_v0147_ab.py tests/test_v0148.py tests/test_v0149.py tests/test_v0150.py tests/test_v0151.py tests/test_v0151_kc.py tests/test_v0152.py tests/test_v0152_wa.py tests/test_v0152_wb.py tests/test_v0153.py tests/test_v0153_wf.py tests/test_v0153_wc.py tests/test_v0154.py tests/test_v0154_b.py tests/test_v0154_c.py tests/test_v01541.py tests/test_v0155.py tests/test_v0156.py tests/test_v0156_d9.py tests/test_v0157.py tests/test_v0158.py tests/test_v0159.py tests/test_v0160.py tests/test_v0161.py tests/test_v0162.py tests/test_v0163.py tests/test_v0164.py tests/test_v0165.py tests/test_v0166.py tests/test_v0167.py
quick: venv
	$(PY) -m pytest $(QUICK) -k "not live and not judge_web and not against_the_protobuf_fake and not real_wire and not dir_source and not twenty_scout_runs"

integration: venv check-mosquitto
	$(PY) -m pytest tests/test_integration_b1.py tests/test_integration_sil1.py tests/test_integration_stall.py

bench: venv check-mosquitto
	scripts/run_bench.sh $(BENCH_ARGS)

venv-sim: venv
	$(VENV)/bin/pip install -r requirements-sim.txt

sim: venv check-mosquitto
	scripts/run_sim.sh $(SIM_ARGS)

stop:
	scripts/stop_bench.sh $(if $(WIPE),--wipe,)

logs:
	@d=$$(cat logs/bench/current_run 2>/dev/null); [ -n "$$d" ] || { echo "no current run"; exit 1; }; tail -n 30 -F $$d/*.log

clean:
	rm -rf .pytest_cache logs/bench logs/*.sqlite build dist *.egg-info
	find . -name __pycache__ -type d -prune -exec rm -rf {} +

distclean: clean
	rm -rf $(VENV)
