import robot

if __name__ == "__main__":
    # We maken een BotData object aan.
    # Dit object bevat alle, veel veranderende, data, zoals de positie, rotatie, grijperstatus, etc.
    data = robot.BotData()
    
    # We maken een BotComponents object aan, met de default componenten.
    # Dit zijn een aantal component klassen die de ROS services en pins van de robot weg 'abstracten',
    # zodat we ze makkelijker kunnen gebruiken en testen.
    components = robot.BotComponents.default_components()
    
    # We maken een Bot object aan, die de data en componenten krijgt.
    # Deze Bot object is de klasse die de robot bestuurt,
    # en de state machine bevat.
    bot = robot.Bot(data, components)

    # We starten de robot. Dit start de interne loop van de robot, die de state machine draait.
    bot.run()
