import gpiozero
from gpiozero import Button
from time import sleep


class Selection_zone():
    def __init__(self, pin_tirette, pin_zone_selector) -> None:
        self.tirette = Button(pin_tirette)
        self.selection_zone = Button(pin_zone_selector)

        self.zone = ""

    def wait_start_loop(self):
        while self.tirette.is_pressed:
            if self.selection_zone.is_pressed:
                self.zone = "jaune"
                print("Zone selectionnée :", self.zone)
            else:
                self.zone = "bleu"
                print("Zone selectionnée :", self.zone)
            sleep(0.1)

        while not(self.tirette.is_pressed):
            if self.selection_zone.is_pressed:
                self.zone = "jaune"
                print("Zone selectionnée :", self.zone)
            else:
                self.zone = "bleu"
                print("Zone selectionnée :", self.zone)
            sleep(0.1)



if __name__ == "__main__":
    
    selection_zone = Selection_zone(14, 15)
    selection_zone.wait_start_loop()

    print("Zone de départ :", selection_zone.zone)

