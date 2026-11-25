# Commands used on 2026-11-25

All commands ran from the laptop. The robot is `pi@10.42.0.218`; the vendor
container is `ArmPiUltra`. `sudo -n` was already authorized on the Pi.

## Deploy

```bash
cd ~/jcheng/jcheng-perception
tar czf - -C deploy systemd |
  ssh pi@10.42.0.218 'rm -rf /tmp/jcheng-perception-systemd && \
    mkdir -p /tmp/jcheng-perception-systemd && \
    tar xzf - -C /tmp/jcheng-perception-systemd'

ssh pi@10.42.0.218 \
  'sudo bash /tmp/jcheng-perception-systemd/systemd/install-split-systemd.sh'

ssh pi@10.42.0.218 'sudo systemctl start jcheng-perception.target'
```

The installer enables `jcheng-perception.target` at boot and seeds
`/etc/jcheng-perception/perception.env` only when it does not exist.

## State and process checks

```bash
ssh pi@10.42.0.218 '
  systemctl list-units --all "jcheng-perception*";
  for u in jcheng-perception-ingress@d435i_ext.service \
           jcheng-perception-ingress@aurora_wrist.service \
           jcheng-perception-fusion.service \
           jcheng-perception-wrist-tf.service \
           jcheng-perception-ext-tf.service; do
    printf "%s active=%s nrestarts=%s result=%s\n" "$u" \
      "$(systemctl is-active $u)" "$(systemctl show -p NRestarts --value $u)" \
      "$(systemctl show -p Result --value $u)";
  done;
  docker exec ArmPiUltra ps -eo pid,args |
    grep -E "lib/jcheng_perception_edge/(perception_edge|perception_fusion|wrist_tf|ext_tf)"'
```

## Topic / TF checks (inside the container)

```bash
docker exec -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml \
  ArmPiUltra bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/jcheng-perception/ros2_ws/install/setup.bash
    ros2 topic echo --once --qos-reliability best_effort /perception/status
    timeout 8 ros2 run tf2_ros tf2_echo base_link camera_optical_measured
    timeout 8 ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame'
```

## Stop propagation

```bash
ssh pi@10.42.0.218 '
  sudo systemctl stop jcheng-perception.target
  # poll is-active until all five are inactive
  ls /tmp/jcheng-perception-*.stopping 2>/dev/null | wc -l   # 0
  sudo ls /var/lib/jcheng-perception/ | wc -l                # 0
  docker exec ArmPiUltra pgrep -fc "lib/jcheng_perception_edge" || true
  sudo systemctl start jcheng-perception.target'
```

Expected wrapper log line (journal):

```text
stop requested: container exit 1 treated as a normal stop
```

## Restart budget and alert (camera-less instance)

```bash
ssh pi@10.42.0.218 '
  sudo systemctl start jcheng-perception-ingress@selftest.service
  until systemctl is-failed --quiet \
      jcheng-perception-ingress@selftest.service; do sleep 1; done
  systemctl show -p NRestarts -p Result \
    jcheng-perception-ingress@selftest.service
  sudo cat /var/lib/jcheng-perception/last-alert.txt
  sudo journalctl -p err -t jcheng-perception -n 5
  sudo systemctl reset-failed jcheng-perception-ingress@selftest.service
  sudo systemctl stop jcheng-perception-ingress@selftest.service'
```

Expected: `NRestarts=10`, `Result=exit-code`, `failed` after ~60 s, alert file
contains `label=ingress-selftest` and the reset/start recovery action.

## Local checks before deploying

```bash
cd ~/jcheng/jcheng-perception
python3 -m pytest tests/test_systemd_deploy.py -q
bash -n deploy/systemd/*.sh
# systemd-analyze verify on copies with ExecStart replaced by /bin/true
```
