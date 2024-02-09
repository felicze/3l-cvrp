from colormap import rgb2hex, rgb2hls, hls2rgb


# https://stackoverflow.com/a/56730252
def sns_list_to_rgb(color_list):
    new_colors = []
    for color in color_list:
        new_color = (color[0] * 255, color[1] * 255, color[2] * 255)
        new_colors.append(new_color)

    return new_colors


def sns_list_to_rgb_string(color_list):
    new_colors = []
    for color in color_list:
        new_color = (color[0] * 255, color[1] * 255, color[2] * 255)
        new_colors.append(f"rgb({new_color[0]},{new_color[1]},{new_color[2]})")

    return new_colors


def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip("#")
    hex_length = len(hex_color)
    return tuple(int(hex_color[i : i + hex_length // 3], 16) for i in range(0, hex_length, hex_length // 3))


def adjust_color_lightness(r, g, b, factor):
    h, l, s = rgb2hls((r / 255), (g / 255), (b / 255))  # noqa: E741
    l = max(min(l * factor, 1.0), 0.0)  # noqa: E741
    r, g, b = hls2rgb(min(h, 1.0), min(l, 1.0), min(s, 1.0))
    return rgb2hex(int(r * 255), int(g * 255), int(b * 255))


def darken_color(r, g, b, factor=0.1):
    return adjust_color_lightness(r, g, b, 1 - factor)


def hex_color_to_shaded_list(hex_color, number_of_colors):
    new_colors = []
    r, g, b = hex_to_rgb(hex_color)
    for i in range(number_of_colors):
        new_color = darken_color(r, g, b, i / number_of_colors)
        new_colors.append(new_color)

    return new_colors
