# Comment faire fonctionner l'écran

Pour cela, je me base sur le [tutoriel](https://raspberrypi.stackexchange.com/questions/53931/why-is-my-raspberry-pi-3-display-not-filling-the-screen).



1. `sudo nano /boot/config.txt`

2. `# disable_overscan=1` (leave this as it is)

3. remove `#` from the following and change the values.

```
overscan_left=0 (Value must be a 0)
overscan_right=480 (only adjust this for right and left movement).
overscan_top=0 (value must be 0 don't try to change it)
overscan_bottom=220 (adjust this for top and bottom).
```
Negative values never worked and I don't know who came up with that.

4. You then need to remove # from resolution and change it to what ever your screen is. Mine is 5" 800 x 480 for better viewing.
```
framebuffer_width=800 (remove the # and change to reasonable resolution)
framebuffer_height=480 (remove the # and change to reasonable resolution)
```
5. MOST IMPORTANT: You need to change hdmi_mode to 4. (leave group the same don't remove the # from group or anything that I haven't).
```
hdmi_mode=4 (Remove the # and change to 4) 
```

Maintenant quitter l'éditeur et effectuer un reboot.
