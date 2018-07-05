# opiservo
Kernel module for orange pi one. Controlled servo ppm (Pulse-position modulation).

Tested and developed for SoC Orange PI One. Uses linux 3.4.113+ with sunxi drivers. 

It will probably work on all platforms using CPU Allwinner H3.

The patch of the sunxi dma driver is used. It is very important. Without this driver it will not work normally.

For precise pauses, uart3 is used. 

I tested the use of only one register PA, which are GPIO 2, 0, 3, 7, 21, 22, 23, 24, 6, 9, 8, 15, 16, 31, 30, 25, 11.

Example for use from php: 

```php
<?php
$h = fopen('/dev/opiservo','w');
$w = 100;
$cmd = "";
// init GPIO use at servo
foreach(array(2,0,3,7,21,22,23,24,6,9,8,15,16,31,30,25,11) as $pin) {
        $cmd .= "$pin=SERVO\n";
        $w+=100;
}
fwrite($h, $cmd);
$cmd = "";
// set ppm value at us
foreach(array(2,0,3,7,21,22,23,24,6,9,8,15,16,31,30,25,11) as $pin) {
        $cmd .= "$pin=$w\n";
        $w+=100;
}
fwrite($h, $cmd);
fclose($h);
?>

```
Screenshots from my Logic Analyzator:
![first pins from array from 100-800us](https://raw.githubusercontent.com/sergey-sh/opiservo/master/screenshot/opiservo_8pins.png)

Scaled:
![scaled 100us](https://raw.githubusercontent.com/sergey-sh/opiservo/master/screenshot/opiservo_8pins_detail.png)

