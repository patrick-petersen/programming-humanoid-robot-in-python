until simspark; do
    echo "simspark 'myserver' crashed with exit code $?.  Respawning.." >&2
    sleep 1
done
