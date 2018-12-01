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

