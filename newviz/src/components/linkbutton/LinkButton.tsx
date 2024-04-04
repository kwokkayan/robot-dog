import { Button } from "@mui/material";
import { Link } from "react-router-dom";

function LinkButton({ color, href, label }) {
  return (
    <Button color={color} component={Link} href={href} to={href}>
      {label}
    </Button>
  );
}
export default LinkButton;
