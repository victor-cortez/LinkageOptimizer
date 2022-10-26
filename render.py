import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
import time

video_name = "videos/" + time.strftime("%Y%m%d-%H%M%S") + "-linkage_video.mp4"
data = pd.read_csv("example2ad.csv")
show_locus = False

plt.clf()
Figure = plt.figure()
plt.axis('equal')


# creating a plot
if show_locus:
    plt.plot(data["xo"], data["yo"])
    plt.plot(data["xi"], data["yi"])
    plt.plot(data["xc"], data["yc"])
    plt.plot(data["xct"], data["yct"])
    plt.plot(data["xot"], data["yot"])

crank_link = plt.plot([data["xi"][0], data["xc"][0]], [
                      data["yi"][0], data["yc"][0]])[0]
crank_header_link = plt.plot([data["xc"][0], data["xct"][0]], [
                             data["yc"][0], data["yct"][0]])[0]
output_header_link = plt.plot([data["xo"][0], data["xot"][0]], [
                              data["yo"][0], data["yot"][0]])[0]
output_link = plt.plot([data["xo"][0], data["xb"][0]], [
                       data["yo"][0], data["yb"][0]])[0]
coupler_link = plt.plot([data["xo"][0], data["xc"][0]], [
                        data["yo"][0], data["yc"][0]])[0]
coupler_top_link = plt.plot([data["xot"][0], data["xct"][0]], [
                            data["yot"][0], data["yct"][0]])[0]


# function takes frame as an input
def AnimationFunction(frame):
    crank_link.set_data(([data["xi"][frame], data["xc"][frame]], [
                        data["yi"][frame], data["yc"][frame]]))
    crank_header_link.set_data(([data["xc"][frame], data["xct"][frame]], [
                               data["yc"][frame], data["yct"][frame]]))
    output_header_link.set_data(([data["xo"][frame], data["xot"][frame]], [
                                data["yo"][frame], data["yot"][frame]]))
    output_link.set_data(([data["xo"][frame], data["xb"][frame]], [
                         data["yo"][frame], data["yb"][frame]]))
    coupler_link.set_data(([data["xo"][frame], data["xc"][frame]], [
                          data["yo"][frame], data["yc"][frame]]))
    coupler_top_link.set_data(([data["xot"][frame], data["xct"][frame]], [
                              data["yot"][frame], data["yct"][frame]]))


anim_created = FuncAnimation(
    Figure, AnimationFunction, frames=len(data), interval=30)

video = anim_created.save(video_name, fps=30, dpi=100,
                          extra_args=['-vcodec', 'libx264'])

# good practice to close the plt object.
plt.close()
