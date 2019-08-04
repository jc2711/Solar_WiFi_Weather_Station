CREATE TABLE `api` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `userid` int(11) NOT NULL,
  `key` varchar(128) NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_api_1_idx` (`userid`),
  CONSTRAINT `fk_api_1` FOREIGN KEY (`userid`) REFERENCES `users` (`id`) ON DELETE NO ACTION ON UPDATE NO ACTION
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

CREATE TABLE `data` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `apiId` int(11) NOT NULL,
  `date` datetime NOT NULL,
  `rel_pressure` double DEFAULT NULL,
  `measured_temp` double DEFAULT NULL,
  `measured_humi` double DEFAULT NULL,
  `volt` double DEFAULT NULL,
  `measured_pres` double DEFAULT NULL,
  `DewpointTemperature` double DEFAULT NULL,
  `HeatIndex` double DEFAULT NULL,
  `ZambrettiLetter` varchar(2) DEFAULT NULL,
  `accuracy_in_percent` double DEFAULT NULL,
  `DewPointSpread` double DEFAULT NULL,
  `zambrettiValue` int(11) DEFAULT NULL,
  `zambrettiTrend` int(11) DEFAULT NULL,
  `temp2` double DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_data_1_idx` (`apiId`),
  CONSTRAINT `fk_data_1` FOREIGN KEY (`apiId`) REFERENCES `api` (`id`) ON DELETE NO ACTION ON UPDATE NO ACTION
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

CREATE TABLE `users` (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `username` varchar(45) NOT NULL,
  `password` varchar(128) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;

