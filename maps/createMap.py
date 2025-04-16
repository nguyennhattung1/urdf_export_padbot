#!/usr/bin/env python3

def create_simple_map(width, height, filename):
    """Create a simple map with walls around the edges and some obstacles"""
    # Create empty map with all free space (255 is free in PGM)
    map_data = [[255 for _ in range(width)] for _ in range(height)]
    
    # Add walls around the edges (0 is occupied in PGM)
    for x in range(width):
        map_data[0][x] = 0  # Top wall
        map_data[height-1][x] = 0  # Bottom wall
    
    for y in range(height):
        map_data[y][0] = 0  # Left wall
        map_data[y][width-1] = 0  # Right wall
    
    # Add some obstacle boxes
    # Box 1: top-left
    for y in range(20, 40):
        for x in range(20, 40):
            map_data[y][x] = 0
    
    # Box 2: top-right
    for y in range(20, 40):
        for x in range(width-40, width-20):
            map_data[y][x] = 0
    
    # Box 3: bottom-left
    for y in range(height-40, height-20):
        for x in range(20, 40):
            map_data[y][x] = 0
    
    # Box 4: bottom-right
    for y in range(height-40, height-20):
        for x in range(width-40, width-20):
            map_data[y][x] = 0
    
    # Box 5: center
    for y in range(height//2-20, height//2+20):
        for x in range(width//2-20, width//2+20):
            map_data[y][x] = 0
    
    # Write PGM file
    with open(filename, 'w') as f:
        # Write PGM header
        f.write('P2\n')  # Plain PGM format
        f.write(f'{width} {height}\n')
        f.write('255\n')  # Max value
        
        # Write map data
        for y in range(height):
            for x in range(width):
                f.write(f'{map_data[y][x]} ')
            f.write('\n')
    
    print(f"Map saved to {filename}")

if __name__ == "__main__":
    # Create a 400x400 map
    create_simple_map(400, 400, "map.pgm") 