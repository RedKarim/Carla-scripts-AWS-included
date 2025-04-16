import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Print all available maps
available_maps = client.get_available_maps()
print("Available Maps:")
for map_name in available_maps:
    print("-", map_name)

# Load desired map
client.load_world('Town04_Opt')

# Get world and set sunny weather
world = client.get_world()
weather = carla.WeatherParameters.ClearNoon
world.set_weather(weather)
