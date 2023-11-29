docker exec -w "/rover-v2/webui/" ros2_rover bash -c "pkill http-server && http-server ."
DISPLAY=:0 chromium-browser -kiosk --incognito http://10.10.20.152:8080