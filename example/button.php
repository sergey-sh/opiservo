<?php
$pins = array(4);
$h = fopen('/dev/opiservo','w');
$cmd = "";
// init GPIO use at button
foreach($pins as $pin) {
        $cmd.= "$pin=BUTTON\n";
}
fwrite($h, $cmd);
fclose($h);

while(true) {
	$data = get_pin_state();
	foreach($pins as $pin) {
		if(isset($data[$pin]) && $data[$pin]=="T") {
			echo "$pin:Off\n";
		} else {
			echo "$pin:On\n";
		}
	}
	sleep(1);
}

function get_pin_state() {
	$r = array();
	foreach(explode("\n",file_get_contents('/dev/opiservo')) as $v) {
		$d = explode("=",$v);
		if(isset($d[0]) && isset($d[1])) {
			$r[$d[0]] = $d[1];
		}
	}
	return $r;
}
	
?>
