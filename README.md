Le repo sert de base commune pour définir les messages du bus CAN du robot Krabi

Il y a un [enum "can_ids"](https://github.com/VictorDubois/CanStruct/blob/main/can_structs.h#L6) => ce sont directement les IDs CAN.
Plus l'ID est faible, plus il est prioritaire.

En commentaire, est précisé le nom de la structure du message.

Elles sont spécifiées dans la deuxième section, sous forme de struct ([ex : ServoMessage](https://github.com/VictorDubois/CanStruct/blob/main/can_structs.h#L90))

Elles font exactement 8 octets (quitte à inclure des champs "_unused")
En effet, le CAN c'est maximum 8 octets (seul le STM32 est compatible CAN-FD).
