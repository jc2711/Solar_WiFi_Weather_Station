<?php

$mysql_host = "HOST";
$mysql_user = "USER";
$mysql_pw = "PASSWD";
$mysql_db = "DB";

class WeatherDB extends mysqli {
    private $apiId = -1;

    public function __construct($host, $user, $pass, $db) {
        parent::__construct($host, $user, $pass, $db);

        if(mysqli_connect_error()) {
            die("Connect Error " . mysqli_connect_error() . "\n");
        }
    }

    public function apiKeyValid($key) {
        $query = sprintf("select id from api where `key`='%s'", $this->real_escape_string($key));
        if($result = $this->query($query)) {
            if($row = $result->fetch_assoc()) {
                $this->apiId = $row["id"];
                return true;
            }
            $result->close();
        }
        return false;
    }

    public function inertData($rel_pressure, $measured_temp, $measured_humi, $volt, $measured_pres, $dewpointTemperature, $heatIndex, $zambrettiLetter, $accuracy_in_percent, $dewPointSpread) {
        $query = sprintf("insert into data (apiId, date, rel_pressure, measured_temp, measured_humi, volt, measured_pres, DewpointTemperature, HeatIndex, ZambrettiLetter, accuracy_in_percent, DewPointSpread) values (%d, now(), %f, %f, %f, %f, %f, %f, %f, '%s', %f, %f)",
            $this->apiId,
            $rel_pressure,
            $measured_temp,
            $measured_humi,
            $volt,
            $measured_pres,
            $dewpointTemperature,
            $heatIndex,
            $this->real_escape_string($zambrettiLetter),
            $accuracy_in_percent,
            $dewPointSpread
            );
        $this->query($query);
    }
}

function getValue($key) {
    if(array_key_exists($key, $_GET)) {
        return $_GET[$key];
    }
    return NULL;
}

$weatherdb = new WeatherDB($mysql_host, $mysql_user, $mysql_pw, $mysql_db);

$apikey = "";
if(array_key_exists("api_key", $_GET)) {
    $apikey = $_GET["api_key"];
}

if($apikey == "") die("no key\n");

if(!$weatherdb->apiKeyValid($apikey)) die("invalid key\n");

$rel_pressure        = getValue("field1");
$measured_temp       = getValue("field2");
$measured_humi       = getValue("field3");
$volt                = getValue("field4");
$measured_pres       = getValue("field5");
$dewpointTemperature = getValue("field6");
$heatIndex           = getValue("field7");
$zambrettiLetter     = getValue("field8");
$accuracy_in_percent = getValue("field9");
$dewPointSpread      = getValue("field10");

$weatherdb->inertData($rel_pressure, $measured_temp, $measured_humi, $volt, $measured_pres, $dewpointTemperature, $heatIndex, $zambrettiLetter, $accuracy_in_percent, $dewPointSpread);

?>
