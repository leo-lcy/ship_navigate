from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QPainter, QColor, QTransform, QPolygonF
from PySide6.QtWidgets import QWidget


class ShipWidget(QWidget):
    def __init__(self, size: int, x: int, y: int, head: int, color: QColor, parent=None):
        super().__init__(parent)
        self.color = color
        self.x = x
        self.y = y
        self.head = head
        self.setFixedSize(size, size)
        self.move(x, y)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QColor(self.color))
        painter.setPen(Qt.PenStyle.NoPen)
        size = min(self.width(), self.height())

        center_x = round(self.rect().x() + size / 2)
        center_y = round(self.rect().y() + size / 2)

        transform = QTransform()
        transform.translate(center_x, center_y)
        transform.rotate(self.head)
        transform.translate(-center_x, -center_y)

        painter.setTransform(transform)

        # 画类似船形的图形，可以用QPolygonF来构建
        # 这里定义一个简单的船形图案（可以根据需要进行调整）
        points = [
            QPointF(center_x - size / 2, center_y + size / 4),  # 左下角
            QPointF(center_x + size / 2, center_y + size / 4),  # 右下角
            QPointF(center_x + size / 4, center_y - size / 4),  # 右上角
            QPointF(center_x - size / 4, center_y - size / 4)   # 左上角
        ]

        # 创建多边形并绘制
        polygon = QPolygonF(points)
        painter.drawPolygon(polygon)

        # 可选：绘制船头部分的指向线
        painter.setPen(QColor("#333333"))
        painter.drawLine(center_x, center_y, center_x, center_y - size / 2)