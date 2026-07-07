#!/usr/bin/env bash
# Benchmark headless del pick-and-place: lancia N run in xvfb, classifica
# l'esito di ognuno leggendo il log, produce un CSV e un riepilogo.
set -u

# ---- Configurazione ----
N_RUNS="${1:-20}"              # numero di run (default 20, override: ./benchmark.sh 30)
RUN_TIMEOUT="${2:-180}"        # timeout per run in secondi (default 180)
WORLD="/home/duccio/ros2_ws/ROS2_project_franka/install/franka_gazebo_bringup/share/franka_gazebo_bringup/worlds/my_world2.sdf"
OUTDIR="benchmark_results/$(date +%Y%m%d_%H%M%S)"
CSV="$OUTDIR/results.csv"

mkdir -p "$OUTDIR"
echo "run_id,esito,durata_s" > "$CSV"

echo "=== Benchmark: $N_RUNS run, timeout ${RUN_TIMEOUT}s/run ==="
echo "=== Output in: $OUTDIR ==="

# Contatori
declare -A COUNT
COUNT[SUCCESS_DIRETTO]=0
COUNT[SUCCESS_RECOVERY]=0
COUNT[FAILURE]=0
COUNT[NO_CUBE]=0
COUNT[TIMEOUT_CRASH]=0

cleanup_processes() {
  # Pulizia aggressiva tra un run e l'altro: uccide tutto cio' che potrebbe
  # restare vivo e contaminare il run successivo.
  pkill -9 -f "gz sim"          2>/dev/null
  pkill -9 -f "ign gazebo"      2>/dev/null
  pkill -9 -f "ruby.*gz"        2>/dev/null
  pkill -9 -f "move_group"      2>/dev/null
  pkill -9 -f "rviz2"           2>/dev/null
  pkill -9 -f "planner_bt"      2>/dev/null
  pkill -9 -f "detector_node"   2>/dev/null
  pkill -9 -f "robot_state_pub" 2>/dev/null
  pkill -9 -f "ros_gz"          2>/dev/null
  pkill -9 -f "parameter_bridge" 2>/dev/null
  pkill -9 -f "spawner"         2>/dev/null
  pkill -9 -f "Xvfb"            2>/dev/null
  sleep 3
}

for i in $(seq 1 "$N_RUNS"); do
  LOG="$OUTDIR/run_$(printf '%03d' "$i").log"
  echo -n "[$(date +%H:%M:%S)] Run $i/$N_RUNS ... "
  START=$(date +%s)

  # Lancia il task headless in xvfb, log su file, in un process group isolato.
  setsid xvfb-run -a -s "-screen 0 1280x1024x24" \
    ros2 launch cube_planner bt_demo.launch.py > "$LOG" 2>&1 &
  LAUNCH_PID=$!

  # Polling: aspetta il verdetto nel log, o il timeout.
  ESITO="TIMEOUT_CRASH"
  while true; do
    NOW=$(date +%s)
    ELAPSED=$((NOW - START))
    if [ "$ELAPSED" -ge "$RUN_TIMEOUT" ]; then
      ESITO="TIMEOUT_CRASH"; break
    fi
    if grep -q "Risultato: SUCCESS" "$LOG" 2>/dev/null; then
      # SUCCESS_RECOVERY solo se c'e' stato un fallimento REALE di uno step
      # (fallback su OMPL, piano fallito, ecc.). Il tag [RECOVERY] da solo NON
      # basta: il GoHome di fine sequenza lo stampa sempre, anche senza recovery.
      if grep -Eq "fallback movePose|FALLITO|piano fallito|Unable to sample|non valida|Esecuzione cartesian .* FALLITA" "$LOG" 2>/dev/null; then
        ESITO="SUCCESS_RECOVERY"
      else
        ESITO="SUCCESS_DIRETTO"
      fi
      break
    fi
    if grep -q "Risultato: FAILURE" "$LOG" 2>/dev/null; then
      ESITO="FAILURE"; break
    fi
    if grep -q "Nessuna posa cubo, esco" "$LOG" 2>/dev/null; then
      ESITO="NO_CUBE"; break
    fi
    sleep 2
  done

  END=$(date +%s)
  DUR=$((END - START))

  # Chiudi questo run: uccidi il process group del launch, poi pulizia globale.
  kill -TERM -"$LAUNCH_PID" 2>/dev/null
  sleep 2
  cleanup_processes

  COUNT[$ESITO]=$(( ${COUNT[$ESITO]} + 1 ))
  echo "$i,$ESITO,$DUR" >> "$CSV"
  echo "$ESITO (${DUR}s)"
done

# ---- Riepilogo ----
echo ""
echo "=========== RIEPILOGO ($N_RUNS run) ==========="
TOT_SUCCESS=$(( ${COUNT[SUCCESS_DIRETTO]} + ${COUNT[SUCCESS_RECOVERY]} ))
echo "SUCCESS totali:    $TOT_SUCCESS / $N_RUNS"
echo "  - diretti:       ${COUNT[SUCCESS_DIRETTO]}"
echo "  - via recovery:  ${COUNT[SUCCESS_RECOVERY]}"
echo "FAILURE:           ${COUNT[FAILURE]}"
echo "NO_CUBE:           ${COUNT[NO_CUBE]}"
echo "TIMEOUT/CRASH:     ${COUNT[TIMEOUT_CRASH]}"
if [ "$N_RUNS" -gt 0 ]; then
  RATE=$(awk "BEGIN{printf \"%.1f\", 100*$TOT_SUCCESS/$N_RUNS}")
  echo "Success rate:      ${RATE}%"
fi
echo "CSV: $CSV"
echo "==============================================="
