from PIL import Image, ImageDraw, ImageFont

# Load the image
image_path = "../default_page.png"
img = Image.open(image_path)
draw = ImageDraw.Draw(img)
# Get image dimensions
width, height = img.size
# Define colors for annotations
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
# Try to use a default font, fallback to default if not available
try:
    font = ImageFont.truetype("msyh.ttc", 16)
    small_font = ImageFont.truetype("msyh.ttc", 14)
except:
    font = ImageFont.load_default()
    small_font = ImageFont.load_default()
# Define regions based on visual analysis
regions = [
    {
        "name": "标题栏",
        "coords": (0, 0, width, 37),
        "color": RED,
        "description": "窗口标题和控制按钮",
    },
    {
        "name": "侧边工具栏",
        "coords": (0, 39, 54, height),
        "color": BLUE,
        "description": "左侧工具面板",
    },
    {
        "name": "工作区",
        "coords": (56, 39, 409, height),
        "color": YELLOW,
        "description": "脚本编辑、终端、形状信息、软件信息",
    },
    {
        "name": "3D视图区",
        "coords": (411, 39, width, height),
        "color": MAGENTA,
        "description": "3D模型预览区域",
    },
]
# Draw rectangles and labels for each region
for i, region in enumerate(regions):
    x1, y1, x2, y2 = region["coords"]
    color = region["color"]

    # Draw rectangle border (thicker line)
    for offset in range(3):
        draw.rounded_rectangle(
            [(x1 + offset, y1 + offset), (x2 - offset, y2 - offset)],
            radius=10,
            outline=color,
            width=1,
        )

    # Add label background
    label_text = region["name"]
    bbox = draw.textbbox((0, 0), label_text, font=font)
    label_width = bbox[2] - bbox[0] + 4
    label_height = bbox[3] - bbox[1] + 4

    # Position label at top-left of region with some padding
    label_x = x1 + int((x2 - x1) / 2 - label_width / 2)
    label_y = y1 + int((y2 - y1) / 2 - label_height / 2)
    if label_x < 0:
        label_x = 0

    # Draw semi-transparent background for label
    label_bg = Image.new("RGBA", (label_width + 10, label_height + 6), (*color, 200))
    img_rgba = img.convert("RGBA")
    img_rgba.paste(label_bg, (label_x, label_y), label_bg)
    img = img_rgba.convert("RGB")
    draw = ImageDraw.Draw(img)

    # Draw label text
    draw.text((label_x + 5, label_y + 3), label_text, fill=WHITE, font=font)
# Add a legend at the bottom or side with descriptions
legend_x = 420
legend_y = 38
legend_title = "界面区域说明:"
draw.text((legend_x, legend_y), legend_title, fill=WHITE, font=font)
y_offset = legend_y + 25
for region in regions:
    # Draw small color box
    draw.rectangle(
        [(legend_x, y_offset), (legend_x + 15, y_offset + 15)],
        fill=region["color"],
        outline=WHITE,
    )
    # Add description text
    desc_text = f"{region['name']}: {region['description']}"
    draw.text((legend_x + 20, y_offset), desc_text, fill=WHITE, font=small_font)
    y_offset += 25

# Save the annotated image
output_path = "../default_page_annotated.png"
img.save(output_path, quality=80)
