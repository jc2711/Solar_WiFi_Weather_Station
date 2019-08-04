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

    public function insertData($rel_pressure, $measured_temp, $measured_humi, $volt, $measured_pres, $dewpointTemperature, $heatIndex, $zambrettiLetter, $accuracy_in_percent, $dewPointSpread, $zambrettiValue, $zambrettiTrend, $temp2) {
        $query = sprintf("insert into data (apiId, date, rel_pressure, measured_temp, measured_humi, volt, measured_pres, DewpointTemperature, HeatIndex, ZambrettiLetter, accuracy_in_percent, DewPointSpread, zambrettiValue, zambrettiTrend, temp2) values (%d, now(), %f, %f, %f, %f, %f, %f, %f, '%s', %f, %f, %d, %d, %f)",
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
            $dewPointSpread,
            $zambrettiValue,
            $zambrettiTrend,
            $temp2
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

$rel_pressure        = getValue("relPres");
$measured_temp       = getValue("measTemp");
$measured_humi       = getValue("measHumi");
$volt                = getValue("volt");
$measured_pres       = getValue("measPres");
$dewpointTemperature = getValue("dewTemp");
$heatIndex           = getValue("heatInd");
$zambrettiValue      = getValue("zambVal");
$accuracy_in_percent = getValue("accuracy");
$dewPointSpread      = getValue("dewSpre");
$zambrettiTrend      = getValue("zambTre");
$temp2               = getValue("temp2");

$weatherdb->insertData($rel_pressure, $measured_temp, $measured_humi, $volt, $measured_pres, $dewpointTemperature, $heatIndex, "", $accuracy_in_percent, $dewPointSpread, $zambrettiValue, $zambrettiTrend, $temp2);

?>
