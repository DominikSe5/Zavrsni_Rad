#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy


class Varijable(object):
    def __init__(self):
        self.pub = rospy.Publisher('/varijabla_funkciji', TipPoruke, queue_size=1)

        rospy.Subscriber('funkcija_varijabli', TipPoruke, self.callback, queue_size=1)
        # PAZI! u subscriberu nema / na pocetku. To znaci da ce topic ispred dobiti namespace nodea,
        # npr. /robot1/funkcija_varijabli

        rospy.spin()

    def callback(self):
        # Od centralnog node-a primis vrijednosti svih R poruka
        # Izracunas svoj novi Q

        self.pub.publish(novi_Q)

if __name__ == "__main__":
    rospy.init_node("node_name")

    try:
        node = Varijable()
    except rospy.ROSInterruptException:
        pass


###############################################################################################################

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy


class Funkcije(object):
    def __init__(self):

        self.Qs = {'agent0': 0, 'agent1': 0}
        self.received = {'agent0': 0, 'agent1': 0}

        rospy.Subscriber('/varijable_funkciji', TipPoruke, self.callback, queue_size=1)

        self.pubs = {}
        for agent in self.Qs:
            self.pubs[agent] = rospy.Publisher('/{}/funkcija_varijabli'.format(agent), TipPoruke, queue_size=1)

        while not rospy.is_shutdown():

            # cekas dok ne dobijes sve poruke, provjeravas self.received

            # racunas R poruke

            # resetiras received.

            for agent in self.Qs:
                self.pubs[agent].publish(self.Qs[agent])


    def callback(self, msg):

        self.Qs[msg.id] = msg.q
        self.received[msg.id] = 0


if __name__ == "__main__":
    rospy.init_node("node_name")

    try:
        node = Funkcije()
    except rospy.ROSInterruptException:
        pass

