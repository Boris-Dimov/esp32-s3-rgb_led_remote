#!/bin/bash

if !(stty -F /dev/ttyACM0 115200 -icrnl min 1 -ixon); then
    echo "Could not initialise /dev/ttyACM0 with 'stty'. Aborting."
    exit 1;
fi

let line=0;
while read -res line < /dev/ttyACM0; do
    case $line in
  "Command=F609")
    #echo "Play"
    dbus-send --print-reply --dest=org.mpris.MediaPlayer2.spotify /org/mpris/MediaPlayer2 org.mpris.MediaPlayer2.Player.PlayPause
    ;;
  "Command=EA15")
    #echo "Next"
    dbus-send --print-reply --dest=org.mpris.MediaPlayer2.spotify /org/mpris/MediaPlayer2 org.mpris.MediaPlayer2.Player.Next
    ;;
  "Command=F807")
    #echo "Previous"
    dbus-send --print-reply --dest=org.mpris.MediaPlayer2.spotify /org/mpris/MediaPlayer2 org.mpris.MediaPlayer2.Player.Previous
    ;;
  *)
    echo $line;
    ;;
esac

done

echo "Our work here is done."
exit 0;
