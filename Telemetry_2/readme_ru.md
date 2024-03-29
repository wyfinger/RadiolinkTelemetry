Этот проект - GPS логгер, предназначенный для установки в радиоправляемые модели, преимущественно самолеты, созданный на базе Arduino Nano (или Pro Mini) и GPS модуля, также потребуется модуль SD карты, любой.

Логгер создан для аппаратуры Radiolink и приемников [*R9D* (*R9DS*)](http://www.radiolink.com.cn/doce/product-detail-120.html) и [*R12D* (*R12DS*)](http://www.radiolink.com.cn/doce/product-detail-126.html), которые имеют телеметрийный порт, таким образом логгер подключается к приемнику радиоуправления, питается от него и передает ему телеметрическую информацию: высота, скорость, пройденная дистанция, верстикальная скорость, напряжение на батарее и GPS координаты.

Практика показала, что в полете смотреть на экран радиопередатчика практически невозможно, поэтому вся эта информация также пишется в GPX лог на SD карточку для последующего анализа (легко импортируется в Google Earth, но дополнительную информацию пока приходится извлекать в Excel, похоже прийдется писать для этого отдельную утилиту).

При потере связи радиоаппаратура Radiolink отображает на экране OSD последние полученные параметры телеметрии, таким образом можно легко найти потерянную модель - достаточно просто цифры широты и долготы вбить в Google и перейти по ссылке на карты.

На SD карте также располагается файл настроек GPS.TXT:

```ini
utc=10         ; time offset
sat=8          ; minimum sattelite to GPS fix and start log
volt=17.93077  ; voltage measurement koeffitient
```

[Вот здесь тема на форуме](https://alnado.ru/forum/viewtopic.php?f=91&t=574), где все начиналось.

