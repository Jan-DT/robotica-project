## Testscripts

| Bestandsnaam | Functie |
| ---          | ---     |
| [test_labeling.py](./test_labeling.py) | Gebruikt om implementatie te testen van een connected component labeling algoritme. |
| [test_mapping.py](./test_mapping.py) | Een scriptje om het mapping-algoritme te testen. Simuleert op een hele simpele manier de sonar. Ook gebruikt voor het plotten van de voorbeelden in het verslag. |
| [test_scan.py](./test_scan.py) | Test scanfunctie. Hier nog geen gebruik gemaakt van een encoder. Produceert een `grid.npy` bestand, aangezien de Orange Pi geen matplotlib geïnstalleerd heeft. Dit `grid.npy` bestand is te plotten met [plotter.py](./plotter.py) |
| [plotter.py](./plotter.py) | Een plotter om `grid.npy` bestanden mee te plotten. |
| [test_grabber.py](./test_grabber.py) | Een scriptje om de grabber mee te testen. Kan grabber openen en sluiten, en meet geleidbaarheid van het object dat het vastheeft. |
| [test_motors.py](./test_motors.py) | Een simpel script om de motoren mee te testen. Rijdt gewoon vooruit. |
| [test_servo.py](./test_servo.py) | Een simpel script om de servo mee te testen. Beweest op en neer. |
| [test_encoder.py](./test_encoder.py) | Een scriptje om de encoder mee te meten. Ik heb het gebruikt met een IR-sensor ipv. een encoder omdat we geen encoders hadden. |
