General idea: error check in message
Bij doorsturen terminaten met \n alleen!

STARTUP:
Arduino stuurt al zijn waarden 1 voor 1 door 
PC bevestigt ontvangst door de ontvangen waarden opnieuw door te sturen.
Arduino bevestigt ontvangst wederom door waarde terug te sturen

SETTINGS NAAR ARDUINO STUREN: TCP
PC stuurt waarde door
Arduino bevestigt door waarde opnieuw door te sturen
PC herstuurt als confirmatie fout is of niet toekomt

MEETWAARDEN VAN ARDUINO: UDP
Arduino stuurt gemeten waarden door, maar wacht niet op bevestiging. 
PC smijt foute waarden gewoon weg

Voorlopig verwacht de arduino enkel bevestiging van RR bij startup
Kan getest worden door bvb RR=5=5 of RR=20= te sturen.