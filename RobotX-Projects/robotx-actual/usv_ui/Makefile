# usv_ui - the boat console kit.  make help | make test | make rehearsal | make stop
ROOT := ..
PY   := $(ROOT)/.venv/bin/python

.PHONY: help test rehearsal stop

help:
	@echo "make test       tests/test_usv_ui.py (share, handover/takeover, cv map, fcu switch, OCS link; needs ../.venv + mosquitto)"
	@echo "make rehearsal  the laptop rehearsal: bench stack + usv_ui on the fake FCU in tmux 'optiprimex'"
	@echo "make stop       stop the rehearsal"
	@echo "the tool:       ../.venv/bin/python usv_ui.py --help   (commands typed inside: help)"

test:
	@[ -x $(PY) ] || { echo "no venv at $(PY): cd $(ROOT) && make venv"; exit 1; }
	cd $(ROOT) && .venv/bin/python -m pytest tests/test_usv_ui.py

# the laptop rehearsal: the bench stack with the mission manager on usv_ui's share (14552), usv_ui on the fake FCU
rehearsal:
	cd $(ROOT) && USV_CONFIG=usv_ui/config/usv_bench_share.yaml scripts/run_bench.sh
	tmux new-window -t optiprimex: -n ui -c $(CURDIR)
	tmux send-keys -t optiprimex:ui "$(PY) usv_ui.py udpin:127.0.0.1:14550 --share udpout:127.0.0.1:14552" C-m
	tmux select-window -t optiprimex:ui
	@echo "tmux attach -t optiprimex   (window ui = usv_ui; cli = ocs_cli; tui = op-tui)"

stop:
	cd $(ROOT) && scripts/stop_bench.sh
