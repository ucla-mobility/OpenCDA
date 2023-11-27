import numpy as np
import matplotlib.pyplot as plt


def draw_box_plt(boxes_dec, ax, color=None, linewidth_scale=1.0):
    """
    draw boxes in a given plt ax
    :param boxes_dec: (N, 5) or (N, 7) in metric
    :param ax:
    :return: ax with drawn boxes
    """
    if not len(boxes_dec)>0:
        return ax
    boxes_np= boxes_dec
    if not isinstance(boxes_np, np.ndarray):
        boxes_np = boxes_np.cpu().detach().numpy()
    if boxes_np.shape[-1]>5:
        boxes_np = boxes_np[:, [0, 1, 3, 4, 6]]
    x = boxes_np[:, 0]
    y = boxes_np[:, 1]
    dx = boxes_np[:, 2]
    dy = boxes_np[:, 3]

    x1 = x - dx / 2
    y1 = y - dy / 2
    x2 = x + dx / 2
    y2 = y + dy / 2
    theta = boxes_np[:, 4:5]
    # bl, fl, fr, br
    corners = np.array([[x1, y1],[x1,y2], [x2,y2], [x2, y1]]).transpose(2, 0, 1)
    new_x = (corners[:, :, 0] - x[:, None]) * np.cos(theta) + (corners[:, :, 1]
              - y[:, None]) * (-np.sin(theta)) + x[:, None]
    new_y = (corners[:, :, 0] - x[:, None]) * np.sin(theta) + (corners[:, :, 1]
              - y[:, None]) * (np.cos(theta)) + y[:, None]
    corners = np.stack([new_x, new_y], axis=2)
    for corner in corners:
        ax.plot(corner[[0,1,2,3,0], 0], corner[[0,1,2,3,0], 1], color=color, linewidth=0.5*linewidth_scale)
        # draw front line (
        ax.plot(corner[[2, 3], 0], corner[[2, 3], 1], color=color, linewidth=2*linewidth_scale)
    return ax


def draw_points_pred_gt_boxes_plt_2d(pc_range, points=None, boxes_pred=None, boxes_gt=None):
    ax = plt.figure(figsize=(14, 4)).add_subplot(1, 1, 1)
    ax.set_aspect('equal', 'box')
    ax.set(xlim=(pc_range[0], pc_range[3]),
           ylim=(pc_range[1], pc_range[4]))
    if points is not None:
        ax.plot(points[:, 0], points[:, 1], 'y.', markersize=0.3)
    if (boxes_gt is not None) and len(boxes_gt)>0:
        ax = draw_box_plt(boxes_gt, ax, color='green')
    if (boxes_pred is not None) and len(boxes_pred)>0:
        ax = draw_box_plt(boxes_pred, ax, color='red')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.show()
    plt.close()


def draw_points_boxes_plt_2d(ax, pc_range, points=None, boxes=None, color=None):
    if points is not None:
        ax.plot(points[:, 0], points[:, 1], '.', markersize=0.3, color=color)
    if (boxes is not None) and len(boxes)>0:
        ax = draw_box_plt(boxes, ax, color=color)

    return ax