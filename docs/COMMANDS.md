# Commandes USB et WebSocket

Ce firmware accepte des commandes sur **USB (CLI)** et **WebSocket**. Les logs et commandes WS sont sur une seule ligne, avec les espaces en percent‑encoding (`%20`) [[memory:8240683]].

## USB (saisie sur le port série)

- wifi status
- wifi portal | wifi portal stop
- wifi connect ssid=<ssid> pass=<pass> save=1
- mqtt status | mqtt on | mqtt off | mqtt connect
- mqtt set host=<h> port=<p> user=<u> pass=<p> topic=<t>
- sensors status | sensors verbose on|off|status
- ws start | ws stop | ws restart
- mlx status | mlx verbose on|off | mlx probe | mlx read reg=0x8000 | mlx speed 100k|400k|1m | mlx power
- ota status | ota set url=<URL> | ota run|start | ota auto on|off | ota verbose on|off|status
- system info

## WebSocket (percent‑encoding)

Exemples (les espaces deviennent `%20`):

- /wifi%20portal
- /wifi%20portal%20stop
- /mqtt%20set?host=192.168.0.10&port=1883&topic=lexaro
- /mqtt%20on
- /mqtt%20connect
- /mqtt%20status
- /ota%20set?url=https%3A%2F%2Fgithub.com%2F<user>%2F<repo>%2Freleases%2Flatest%2Fdownload%2Ffirmware.bin
- /ota%20run
- /ota%20verbose%20on
- /ota%20status
- /ws%20restart

## Logs

Tous les logs utilisent une seule ligne et substituent les espaces par `%20`. Exemple:

```
OTA%20start
OTA%20preflight%20code=302
OTA%20final_url=https://...
OTA%20update%20begin
OTA%20ok
OTA%20rebooting
```


