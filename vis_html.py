from klampt import WorldModel, vis

world = WorldModel()
world.readFile("world.xml")

vis.init("IPython")
vis.add("world", world)
vis.show()
#string = vis.nativeWindow().page()
#with open("vis.html", "w") as f:
#    f.write(string)
