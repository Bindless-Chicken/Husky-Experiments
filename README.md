# Husky Experiments

<div align=center>
<img src="https://www.clearpathrobotics.com/wp-content/uploads/2015/08/husky-essentials-pack.jpg"/>
</div>

This is an experiment space for me to mess around with Clearpath Robotics' Husky.

# Content

## Simple Nav

The simple nav package is a simple experiment at trying to find an object using the onboard laser and driving in its direction.

### Goals

- [X] Moving Husky
- [X] Extract Laser info
- [X] Extract closest and drive towards it
- [X] Emergency stop when in close proximity

## Tunnel Nav

Drive inside a corridor with two defined borders.

### Goals

- [X] Drive in the center of the corridor
- [ ] Handle corners
- [ ] Gracefully handle end of tracks by turning back

## Camera Process

Attach a camera to the husky and drive towards an object of color.

### Goals

- [X] Retrieve a camera feed
- [ ] Find an object of color
- [ ] Drive toward the colored object
