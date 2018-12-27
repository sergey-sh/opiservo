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
	foreach(get_button_state($pins) as $pin=>$state) {
		echo "$pin:$state\n";
	}
	sleep(1);
}

function get_button_state($pin) {
	$data = get_pin_state();
	if(is_array($pin)) {
		$r = array();
		foreach($pin as $v) {
			$r[$v] = (isset($data[$v]) && $data[$v]=="T")?0:1;
		}
		return $r;
	} else {
		return (isset($data[$pin]) && $data[$pin]=="T")?0:1;
	}
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
