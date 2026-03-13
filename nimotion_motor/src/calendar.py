from datetime import datetime, timedelta, timezone
from icalendar import Calendar, Event, rrule

# 创建日历对象
cal = Calendar()
cal.add('prodid', '-//My Calendar//example.com//')
cal.add('version', '2.0')

# 创建重复事件（每周一上午10点）
start_date = datetime(2025, 2, 21, 10, 0, tzinfo=timezone(timedelta(hours=8)))
weekly_rule = rrule(rrule.WEEKLY, byweekday=0, dtstart=start_date)
event = Event(
    summary='周例会',
    dtstart=start_date,
    dtend=start_date + timedelta(hours=2),
    location='线上会议室',
    description='讨论项目进展',
    priority=1,
    rrule=weekly_rule
)
cal.add_component(event)

# 保存文件
with open('my_calendar.ics', 'wb') as f:
    f.write(cal.to_ical())